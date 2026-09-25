"""
spawn_traffic.py
Populates the map with NPC vehicles controlled by CARLA's Traffic Manager.
Tuned to mimic dense, undisciplined Indian traffic (tight gaps, some red-light
running, varied speeds) rather than orderly Western-style traffic.

Run this FIRST before other scripts if you want a populated scene.
"""

import random
import time
from carla_utils import connect

NUM_VEHICLES = 20
RUN_SECONDS = 60


def main():
    client, world = connect()
    blueprint_library = world.get_blueprint_library()

    traffic_manager = client.get_trafficmanager()
    traffic_manager.set_global_distance_to_leading_vehicle(2.5)

    spawn_points = world.get_map().get_spawn_points()
    vehicle_blueprints = blueprint_library.filter('vehicle.*')

    actor_list = []
    random.shuffle(spawn_points)  # avoid collisions from picking the same point twice

    for i in range(min(NUM_VEHICLES, len(spawn_points))):
        bp = random.choice(vehicle_blueprints)
        npc = world.try_spawn_actor(bp, spawn_points[i])
        if npc:
            npc.set_autopilot(True, traffic_manager.get_port())
            actor_list.append(npc)

    print(f"Spawned {len(actor_list)} traffic vehicles")

    # Inject erratic behavior to mimic Indian traffic conditions
    for npc in actor_list:
        traffic_manager.ignore_lights_percentage(npc, 20)          # some run red lights
        traffic_manager.distance_to_leading_vehicle(npc, 1.0)       # tighter gaps
        traffic_manager.vehicle_percentage_speed_difference(npc, random.randint(-30, 30))

    print(f"Running for {RUN_SECONDS} seconds... (Ctrl+C to stop early)")
    try:
        time.sleep(RUN_SECONDS)
    except KeyboardInterrupt:
        pass
    finally:
        for npc in actor_list:
            if npc.is_alive:
                npc.destroy()
        print("Cleaned up traffic vehicles")


if __name__ == '__main__':
    main()
