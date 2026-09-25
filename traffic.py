import carla
import random
import time

client = carla.Client('localhost', 2000)
client.set_timeout(10.0)
world = client.get_world()
blueprint_library = world.get_blueprint_library()

traffic_manager = client.get_trafficmanager()
traffic_manager.set_global_distance_to_leading_vehicle(2.5)

spawn_points = world.get_map().get_spawn_points()
vehicle_blueprints = blueprint_library.filter('vehicle.*')

actor_list = []
for i in range(20):  # spawn 20 NPC vehicles
    bp = random.choice(vehicle_blueprints)
    spawn_point = random.choice(spawn_points)
    npc = world.try_spawn_actor(bp, spawn_point)
    if npc:
        npc.set_autopilot(True, traffic_manager.get_port())
        actor_list.append(npc)

print(f"Spawned {len(actor_list)} traffic vehicles")

# Inject some erratic behavior to mimic Indian traffic chaos
for npc in actor_list:
    traffic_manager.ignore_lights_percentage(npc, 20)  # 20% chance ignores red lights
    traffic_manager.distance_to_leading_vehicle(npc, 1.0)  # tighter gaps
    traffic_manager.vehicle_percentage_speed_difference(npc, random.randint(-30, 30))

time.sleep(60)  # let it run

for npc in actor_list:
    npc.destroy()
