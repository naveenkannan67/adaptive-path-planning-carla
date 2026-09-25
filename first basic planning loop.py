import carla
import random
import time
import math

client = carla.Client('localhost', 2000)
client.set_timeout(10.0)
world = client.get_world()
blueprint_library = world.get_blueprint_library()
map_ = world.get_map()

vehicle_bp = blueprint_library.find('vehicle.tesla.model3')
spawn_points = map_.get_spawn_points()
start_point = random.choice(spawn_points)
vehicle = world.try_spawn_actor(vehicle_bp, start_point)

# Pick a goal waypoint far from the start
goal_point = random.choice(spawn_points)
print(f"Goal: {goal_point.location}")

def distance(loc1, loc2):
    return math.sqrt((loc1.x - loc2.x)**2 + (loc1.y - loc2.y)**2)

for _ in range(300):  # run ~300 ticks
    vehicle_loc = vehicle.get_location()
    dist_to_goal = distance(vehicle_loc, goal_point.location)

    if dist_to_goal < 3.0:
        print("Reached goal!")
        break

    # Check nearby actors for obstacles (very basic collision avoidance)
    nearby_vehicles = world.get_actors().filter('vehicle.*')
    obstacle_close = False
    for other in nearby_vehicles:
        if other.id != vehicle.id:
            if distance(vehicle_loc, other.get_location()) < 5.0:
                obstacle_close = True
                break

    control = carla.VehicleControl()
    if obstacle_close:
        control.throttle = 0.0
        control.brake = 1.0
    else:
        control.throttle = 0.4
        control.steer = 0.0  # replace with real steering-toward-goal math later

    vehicle.apply_control(control)
    time.sleep(0.05)

vehicle.destroy()
