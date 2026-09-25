"""
Adaptive Path Planning and Collision Avoidance for Autonomous Vehicles
on Unstructured Roads -- CARLA implementation.

SIH26037 / Team Tech Vision

Pipeline implemented here, matching the proposal:

  Global planner   -> graph-built A* over the CARLA OpenDRIVE waypoint
                       network (stand-in for the full Hybrid-A* planner;
                       see NOTES.md for how to extend it toward true
                       Hybrid-A* with continuous-curvature expansion).
  Local reactive
  layer            -> potential-field repulsion from nearby dynamic
                       actors (pedestrians / two-wheelers / vehicles),
                       blended into a Stanley-style steering controller
                       and a speed controller that slows near obstacles.
  Safety layer      -> hard brake if any actor enters the collision
                       radius regardless of what the planner/controller
                       are doing.
  Local re-plan     -> if the vehicle is stuck (little progress over a
                       rolling time window), nearby dynamic-obstacle
                       cells are temporarily marked as blocked and a
                       fresh local A* route is spliced in.

Obstacle detection currently uses ground-truth actor poses from
world.get_actors() rather than the attached LiDAR/camera/radar point
clouds -- this is standard for a first working planner+control demo.
The sensors are attached and streaming so a perception module (drivable
-area segmentation, obstacle detection) can be dropped in later without
touching the planner or controller. See NOTES.md.

Run:
    1. Start the CARLA server (CarlaUE4.exe / ./CarlaUE4.sh)
    2. pip install -r requirements.txt   (match the carla version to your server)
    3. python adaptive_path_planning_carla.py --town Town03
"""

import argparse
import collections
import heapq
import math
import random
import sys
import time

try:
    import carla
except ImportError:
    sys.exit(
        "Could not import carla. Install the client library that matches "
        "your CARLA server version, e.g.:\n    pip install carla==0.9.16"
    )

try:
    import pygame
    HAVE_PYGAME = True
except ImportError:
    HAVE_PYGAME = False


# --------------------------------------------------------------------------
# Tunables
# --------------------------------------------------------------------------

SENSOR_RADIUS = 22.0          # m, local-avoidance perception range
COLLISION_RADIUS = 3.5        # m, hard-brake distance
NEAR_MISS_RADIUS = 6.0        # m, logged but not braked
WAYPOINT_SPACING = 3.0        # m, resolution used to sample each road edge
HAZARD_COST_RADIUS = 6.0      # m, soft-cost radius around potholes/debris props
MAX_SPEED_KMH = 30.0
STUCK_WINDOW_S = 2.5
STUCK_DISTANCE_M = 1.5
REPLAN_COOLDOWN_S = 3.0

HAZARD_PROP_BLUEPRINTS = [
    "static.prop.dirtdebris01",
    "static.prop.dirtdebris02",
    "static.prop.dirtdebris03",
    "static.prop.trafficcone01",
]


# --------------------------------------------------------------------------
# Geometry helpers
# --------------------------------------------------------------------------

def flat(loc):
    return (loc.x, loc.y)


def dist2d(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


# --------------------------------------------------------------------------
# Global planner: A* over the OpenDRIVE topology graph
# --------------------------------------------------------------------------

class WaypointGraphPlanner:
    """Builds a routing graph from carla_map.get_topology() and runs A*
    over it. Each topology edge (a lane segment) is re-sampled at
    WAYPOINT_SPACING so the returned route is a dense list of points
    suitable for a tracking controller, not just junction-to-junction.

    This mirrors what CARLA's own GlobalRoutePlanner does, kept
    self-contained here so the script has no dependency on the
    PythonAPI/carla/agents example package.
    """

    def __init__(self, carla_map):
        self.map = carla_map
        self.nodes = {}          # node_key -> (x, y)
        self.edges = collections.defaultdict(list)  # node_key -> [(neighbor_key, cost, points)]
        self.hazard_points = []  # extra soft-cost sources, e.g. potholes/debris
        self._build()

    @staticmethod
    def _key(loc, precision=1.0):
        return (round(loc.x / precision), round(loc.y / precision))

    def _build(self):
        topology = self.map.get_topology()
        for wp_a, wp_b in topology:
            ka = self._key(wp_a.transform.location)
            kb = self._key(wp_b.transform.location)
            self.nodes[ka] = flat(wp_a.transform.location)
            self.nodes[kb] = flat(wp_b.transform.location)

            points = self._sample_edge(wp_a, wp_b)
            cost = self._path_length(points)
            self.edges[ka].append((kb, cost, points))

    def _sample_edge(self, wp_a, wp_b):
        points = [flat(wp_a.transform.location)]
        cur = wp_a
        guard = 0
        target = flat(wp_b.transform.location)
        while dist2d(flat(cur.transform.location), target) > WAYPOINT_SPACING and guard < 500:
            nxts = cur.next(WAYPOINT_SPACING)
            if not nxts:
                break
            cur = nxts[0]
            points.append(flat(cur.transform.location))
            guard += 1
        points.append(target)
        return points

    @staticmethod
    def _path_length(points):
        total = 0.0
        for i in range(1, len(points)):
            total += dist2d(points[i - 1], points[i])
        return total

    def hazard_cost(self, point):
        cost = 0.0
        for hp in self.hazard_points:
            d = dist2d(point, hp)
            if d < HAZARD_COST_RADIUS:
                cost += (HAZARD_COST_RADIUS - d) / HAZARD_COST_RADIUS * 4.0
        return cost

    def _nearest_node(self, loc_xy):
        return min(self.nodes.keys(), key=lambda k: dist2d(self.nodes[k], loc_xy))

    def route(self, start_loc, goal_loc, blocked_nodes=None):
        """A* search over the graph. Returns a flat list of (x, y) points,
        or None if no route was found."""
        blocked_nodes = blocked_nodes or set()
        start_xy = flat(start_loc)
        goal_xy = flat(goal_loc)
        start_node = self._nearest_node(start_xy)
        goal_node = self._nearest_node(goal_xy)

        open_heap = [(0.0, start_node)]
        g_score = {start_node: 0.0}
        came_from = {}
        came_points = {}
        visited = set()

        while open_heap:
            _, node = heapq.heappop(open_heap)
            if node in visited:
                continue
            visited.add(node)
            if node == goal_node:
                break
            for neighbor, cost, points in self.edges.get(node, []):
                if neighbor in blocked_nodes:
                    continue
                extra = sum(self.hazard_cost(p) for p in points) / max(1, len(points))
                tentative = g_score[node] + cost + extra
                if tentative < g_score.get(neighbor, float("inf")):
                    g_score[neighbor] = tentative
                    came_from[neighbor] = node
                    came_points[neighbor] = points
                    f = tentative + dist2d(self.nodes[neighbor], goal_xy)
                    heapq.heappush(open_heap, (f, neighbor))

        if goal_node not in came_from and goal_node != start_node:
            return None

        chain = []
        node = goal_node
        while node != start_node:
            chain.append(came_points[node])
            node = came_from[node]
        chain.reverse()

        flat_path = [start_xy]
        for points in chain:
            flat_path.extend(points)
        flat_path.append(goal_xy)
        return flat_path


# --------------------------------------------------------------------------
# Local controller: potential-field steering + Stanley lateral control
# --------------------------------------------------------------------------

class LocalController:
    def __init__(self, planner):
        self.planner = planner
        self.path = []
        self.path_idx = 0
        self.last_progress_pos = None
        self.last_progress_t = 0.0
        self.replan_cooldown = 0.0
        self.replans = 0
        self.near_misses = 0
        self.distance_travelled = 0.0
        self._seen_near_miss = {}
        self.status = "path"

    def set_path(self, path):
        self.path = path
        self.path_idx = 0

    def _lookahead_target(self, pos, lookahead=8.0):
        i = self.path_idx
        while i < len(self.path) - 1 and dist2d(pos, self.path[i]) < lookahead:
            i += 1
        self.path_idx = i
        return self.path[i]

    def _nearby_dynamic_actors(self, world, ego):
        pos = flat(ego.get_location())
        actors = []
        for actor in world.get_actors():
            if actor.id == ego.id:
                continue
            tid = actor.type_id
            if not (tid.startswith("walker.pedestrian") or tid.startswith("vehicle.")):
                continue
            d = dist2d(pos, flat(actor.get_location()))
            if d < SENSOR_RADIUS:
                actors.append((actor, d))
        return actors

    def compute_control(self, world, ego, sim_time, dt):
        pos = flat(ego.get_location())
        transform = ego.get_transform()
        heading = math.radians(transform.rotation.yaw)
        velocity = ego.get_velocity()
        speed = math.hypot(velocity.x, velocity.y)

        if not self.path or self.path_idx >= len(self.path) - 1:
            if self.path and dist2d(pos, self.path[-1]) < 3.0:
                self.status = "done"
            return carla.VehicleControl(throttle=0.0, steer=0.0, brake=1.0), True

        target = self._lookahead_target(pos)

        nearby = self._nearby_dynamic_actors(world, ego)
        rep_x, rep_y = 0.0, 0.0
        min_dist = float("inf")
        closest_kind = None
        for actor, d in nearby:
            gap = d - 1.0
            if gap < min_dist:
                min_dist = gap
                closest_kind = "pedestrian" if actor.type_id.startswith("walker") else "vehicle"
            if d < SENSOR_RADIUS:
                w = ((SENSOR_RADIUS - d) / SENSOR_RADIUS) ** 2
                apos = flat(actor.get_location())
                dx = (pos[0] - apos[0]) / max(0.001, d)
                dy = (pos[1] - apos[1]) / max(0.001, d)
                rep_x += dx * w
                rep_y += dy * w
            if COLLISION_RADIUS < gap < NEAR_MISS_RADIUS:
                key = actor.id
                if sim_time - self._seen_near_miss.get(key, -99) > 2.5:
                    self.near_misses += 1
                    self._seen_near_miss[key] = sim_time

        to_target = (target[0] - pos[0], target[1] - pos[1])
        t_len = max(0.001, math.hypot(*to_target))
        attractive = (to_target[0] / t_len, to_target[1] / t_len)

        blend_x = attractive[0] + rep_x * 1.6
        blend_y = attractive[1] + rep_y * 1.6
        desired_heading = math.atan2(blend_y, blend_x)

        heading_error = desired_heading - heading
        heading_error = (heading_error + math.pi) % (2 * math.pi) - math.pi
        steer = clamp(heading_error / math.radians(45.0), -1.0, 1.0)

        rep_mag = math.hypot(rep_x, rep_y)
        target_speed_kmh = MAX_SPEED_KMH
        if rep_mag > 0.15:
            target_speed_kmh *= clamp(1.0 - rep_mag * 0.7, 0.2, 1.0)
        if abs(heading_error) > math.radians(35):
            target_speed_kmh *= 0.6

        hard_brake = min_dist < COLLISION_RADIUS
        speed_kmh = speed * 3.6
        if hard_brake:
            throttle, brake = 0.0, 1.0
        elif speed_kmh < target_speed_kmh:
            throttle, brake = clamp((target_speed_kmh - speed_kmh) / 10.0, 0.15, 0.75), 0.0
        else:
            throttle, brake = 0.0, clamp((speed_kmh - target_speed_kmh) / 10.0, 0.0, 0.6)

        self.status = "avoid:" + closest_kind if rep_mag > 0.2 and closest_kind else "path"

        self._track_progress(pos, sim_time)
        self.distance_travelled += speed * dt
        self.replan_cooldown = max(0.0, self.replan_cooldown - dt)

        control = carla.VehicleControl(throttle=float(throttle), steer=float(steer), brake=float(brake))
        return control, False

    def _track_progress(self, pos, sim_time):
        if self.last_progress_pos is None:
            self.last_progress_pos = pos
            self.last_progress_t = sim_time
            return
        if sim_time - self.last_progress_t < STUCK_WINDOW_S:
            return
        moved = dist2d(pos, self.last_progress_pos)
        self.last_progress_pos = pos
        self.last_progress_t = sim_time
        if moved < STUCK_DISTANCE_M and self.replan_cooldown <= 0.0:
            self.status = "replanning"
            self.needs_replan = True

    def maybe_replan(self, world, ego, goal_loc):
        if not getattr(self, "needs_replan", False):
            return False
        self.needs_replan = False
        self.replan_cooldown = REPLAN_COOLDOWN_S
        self.replans += 1

        blocked = set()
        for actor in world.get_actors():
            tid = actor.type_id
            if actor.id == ego.id or not (tid.startswith("walker.pedestrian") or tid.startswith("vehicle.")):
                continue
            aloc = flat(actor.get_location())
            for key, xy in self.planner.nodes.items():
                if dist2d(xy, aloc) < 4.0:
                    blocked.add(key)

        ahead_idx = min(self.path_idx + 12, len(self.path) - 1)
        ahead_point = self.path[ahead_idx]
        new_leg = self.planner.route(ego.get_location(), carla.Location(x=ahead_point[0], y=ahead_point[1], z=0.0), blocked)
        if new_leg:
            self.path = new_leg + self.path[ahead_idx:]
            self.path_idx = 0
            return True
        return False


# --------------------------------------------------------------------------
# Scenario construction
# --------------------------------------------------------------------------

def spawn_ego(world, blueprint_library, spawn_points):
    bp = blueprint_library.find("vehicle.tesla.model3")
    bp.set_attribute("role_name", "ego")
    spawn = spawn_points[0]
    vehicle = world.try_spawn_actor(bp, spawn)
    tries = 1
    while vehicle is None and tries < len(spawn_points):
        vehicle = world.try_spawn_actor(bp, spawn_points[tries])
        tries += 1
    if vehicle is None:
        sys.exit("Could not find a free spawn point for the ego vehicle.")
    return vehicle


def attach_sensors(world, blueprint_library, ego, actor_list):
    """Attaches LiDAR, RGB camera and radar to the ego vehicle. Streaming
    but unused by the planner/controller today -- wire a perception module
    onto these callbacks to replace the ground-truth obstacle lookup."""
    lidar_bp = blueprint_library.find("sensor.lidar.ray_cast")
    lidar_bp.set_attribute("range", "50")
    lidar_bp.set_attribute("rotation_frequency", "20")
    lidar_bp.set_attribute("channels", "32")
    lidar_bp.set_attribute("points_per_second", "200000")
    lidar = world.spawn_actor(lidar_bp, carla.Transform(carla.Location(x=0.0, z=2.2)), attach_to=ego)
    actor_list.append(lidar)

    cam_bp = blueprint_library.find("sensor.camera.rgb")
    cam_bp.set_attribute("image_size_x", "800")
    cam_bp.set_attribute("image_size_y", "500")
    cam_bp.set_attribute("fov", "100")
    camera = world.spawn_actor(cam_bp, carla.Transform(carla.Location(x=1.6, z=1.7)), attach_to=ego)
    actor_list.append(camera)

    radar_bp = blueprint_library.find("sensor.other.radar")
    radar_bp.set_attribute("horizontal_fov", "35")
    radar_bp.set_attribute("range", "40")
    radar = world.spawn_actor(radar_bp, carla.Transform(carla.Location(x=2.0, z=1.0)), attach_to=ego)
    actor_list.append(radar)

    return camera


def spawn_pedestrians(client, world, blueprint_library, count, actor_list):
    walker_bps = blueprint_library.filter("walker.pedestrian.*")
    controllers = []
    spawned = 0
    attempts = 0
    while spawned < count and attempts < count * 6:
        attempts += 1
        loc = world.get_random_location_from_navigation()
        if loc is None:
            continue
        bp = random.choice(walker_bps)
        walker = world.try_spawn_actor(bp, carla.Transform(loc))
        if walker is None:
            continue
        actor_list.append(walker)
        controller_bp = blueprint_library.find("controller.ai.walker")
        controller = world.spawn_actor(controller_bp, carla.Transform(), attach_to=walker)
        actor_list.append(controller)
        controller.start()
        dest = world.get_random_location_from_navigation()
        if dest:
            controller.go_to_location(dest)
        controller.set_max_speed(random.uniform(1.0, 1.8))
        controllers.append((controller, world))
        spawned += 1
    return controllers


def retarget_pedestrians(world, controllers, sim_time, last_retarget):
    if sim_time - last_retarget[0] < 6.0:
        return
    last_retarget[0] = sim_time
    for controller, w in controllers:
        dest = w.get_random_location_from_navigation()
        if dest:
            controller.go_to_location(dest)


def spawn_traffic(world, blueprint_library, traffic_manager, spawn_points, count, actor_list):
    two_wheeler_ids = ["vehicle.diamondback.century", "vehicle.gazelle.omafiets",
                        "vehicle.harley-davidson.low_rider", "vehicle.kawasaki.ninja",
                        "vehicle.yamaha.yzf", "vehicle.vespa.zx125"]
    car_ids = ["vehicle.audi.tt", "vehicle.mini.cooper_s", "vehicle.nissan.micra"]
    candidates = []
    for bid in two_wheeler_ids + car_ids:
        bp = blueprint_library.filter(bid)
        if bp:
            candidates.append(bp[0])
    if not candidates:
        candidates = list(blueprint_library.filter("vehicle.*"))

    random.shuffle(spawn_points)
    spawned = 0
    for sp in spawn_points[1:]:
        if spawned >= count:
            break
        bp = random.choice(candidates)
        vehicle = world.try_spawn_actor(bp, sp)
        if vehicle is None:
            continue
        vehicle.set_autopilot(True, traffic_manager.get_port())
        actor_list.append(vehicle)
        spawned += 1
    return spawned


def scatter_hazard_props(world, blueprint_library, ego_start, planner, count, actor_list):
    bp_lib = blueprint_library
    spawn_locations = random.sample(list(planner.nodes.values()), min(count, len(planner.nodes)))
    placed = []
    for x, y in spawn_locations:
        if dist2d((x, y), flat(ego_start)) < 15.0:
            continue
        bp_name = random.choice(HAZARD_PROP_BLUEPRINTS)
        bps = bp_lib.filter(bp_name)
        if not bps:
            continue
        loc = carla.Location(x=x, y=y, z=0.3)
        actor = world.try_spawn_actor(bps[0], carla.Transform(loc))
        if actor:
            actor_list.append(actor)
            placed.append((x, y))
    planner.hazard_points = placed
    return placed


# --------------------------------------------------------------------------
# Minimal HUD (pygame, optional)
# --------------------------------------------------------------------------

class Hud:
    def __init__(self, width, height):
        self.enabled = HAVE_PYGAME
        if not self.enabled:
            return
        pygame.init()
        pygame.font.init()
        self.display = pygame.display.set_mode((width, height))
        pygame.display.set_caption("Adaptive Path Planning -- CARLA demo (SIH26037)")
        self.font = pygame.font.SysFont("consolas", 16)
        self.surface = None

    def on_camera_image(self, image):
        if not self.enabled:
            return
        import numpy as np
        array = np.frombuffer(image.raw_data, dtype=np.uint8)
        array = array.reshape((image.height, image.width, 4))[:, :, :3][:, :, ::-1]
        self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))

    def render(self, telemetry_lines):
        if not self.enabled:
            return
        if self.surface is not None:
            self.display.blit(self.surface, (0, 0))
        y = 8
        for line in telemetry_lines:
            text = self.font.render(line, True, (255, 255, 255), (0, 0, 0))
            self.display.blit(text, (8, y))
            y += 20
        pygame.display.flip()

    def pump(self):
        if not self.enabled:
            return True
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return False
            if event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                return False
        return True

    def quit(self):
        if self.enabled:
            pygame.quit()


def status_label(status):
    return {
        "path": "Following global path",
        "replanning": "Local re-plan: path blocked, routing around",
        "done": "Goal reached",
    }.get(status, "Avoiding " + status.split(":")[1] if status and status.startswith("avoid:") else status or "idle")


# --------------------------------------------------------------------------
# Main
# --------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("--town", default="Town03")
    parser.add_argument("--pedestrians", type=int, default=14)
    parser.add_argument("--traffic", type=int, default=8)
    parser.add_argument("--hazards", type=int, default=10)
    parser.add_argument("--no-hud", action="store_true")
    args = parser.parse_args()

    client = carla.Client(args.host, args.port)
    client.set_timeout(15.0)
    world = client.load_world(args.town)

    original_settings = world.get_settings()
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = 0.05
    world.apply_settings(settings)

    traffic_manager = client.get_trafficmanager()
    traffic_manager.set_synchronous_mode(True)

    blueprint_library = world.get_blueprint_library()
    carla_map = world.get_map()
    spawn_points = carla_map.get_spawn_points()

    actor_list = []
    hud = None
    try:
        print("Building global route graph from OpenDRIVE topology...")
        planner = WaypointGraphPlanner(carla_map)

        ego = spawn_ego(world, blueprint_library, spawn_points)
        actor_list.append(ego)

        camera = attach_sensors(world, blueprint_library, ego, actor_list)

        hazards = scatter_hazard_props(world, blueprint_library, ego.get_location(), planner, args.hazards, actor_list)
        print("Placed {} hazard props (potholes/debris stand-ins)".format(len(hazards)))

        controllers = spawn_pedestrians(client, world, blueprint_library, args.pedestrians, actor_list)
        print("Spawned {} pedestrians".format(len(controllers)))

        n_traffic = spawn_traffic(world, blueprint_library, traffic_manager, spawn_points, args.traffic, actor_list)
        print("Spawned {} mixed-traffic vehicles/two-wheelers".format(n_traffic))

        goal_point = max(spawn_points, key=lambda sp: dist2d(flat(sp.location), flat(ego.get_location())))
        route = planner.route(ego.get_location(), goal_point.location)
        if route is None:
            sys.exit("No route found between spawn point and goal -- try a different --town.")

        controller = LocalController(planner)
        controller.set_path(route)

        hud = Hud(800, 500) if not args.no_hud else None
        if hud and hud.enabled:
            camera.listen(hud.on_camera_image)

        spectator = world.get_spectator()
        last_retarget = [0.0]
        sim_time = 0.0
        running = True

        print("Simulation running. Ctrl+C or Esc (with HUD) to stop.")
        while running:
            world.tick()
            sim_time += settings.fixed_delta_seconds

            for pt in controller.path[max(0, controller.path_idx):controller.path_idx + 40]:
                world.debug.draw_point(
                    carla.Location(x=pt[0], y=pt[1], z=0.3), size=0.08,
                    color=carla.Color(232, 163, 61), life_time=0.1)

            control, done = controller.compute_control(world, ego, sim_time, settings.fixed_delta_seconds)
            ego.apply_control(control)

            if done and controller.status == "done":
                print("Goal reached. Distance: {:.1f} m, replans: {}, near misses: {}".format(
                    controller.distance_travelled, controller.replans, controller.near_misses))
                break

            controller.maybe_replan(world, ego, goal_point.location)
            retarget_pedestrians(world, controllers, sim_time, last_retarget)

            tf = ego.get_transform()
            spectator.set_transform(carla.Transform(
                tf.location + carla.Location(z=35), carla.Rotation(pitch=-90)))

            if hud:
                lines = [
                    "t = {:.1f}s   {}".format(sim_time, status_label(controller.status)),
                    "distance {:.1f} m   speed {:.1f} km/h".format(
                        controller.distance_travelled, math.hypot(*[getattr(ego.get_velocity(), a) for a in "xy"]) * 3.6),
                    "local replans {}   near misses {}".format(controller.replans, controller.near_misses),
                ]
                hud.render(lines)
                running = hud.pump()
            else:
                if int(sim_time * 10) % 20 == 0:
                    print("t={:.1f}s  {}  dist={:.1f}m  replans={}  near_misses={}".format(
                        sim_time, status_label(controller.status), controller.distance_travelled,
                        controller.replans, controller.near_misses))

    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        if hud:
            hud.quit()
        print("Cleaning up {} actors...".format(len(actor_list)))
        for actor in actor_list:
            try:
                if actor.type_id.startswith("controller."):
                    actor.stop()
                actor.destroy()
            except Exception:
                pass
        world.apply_settings(original_settings)
        traffic_manager.set_synchronous_mode(False)


if __name__ == "__main__":
    main()
