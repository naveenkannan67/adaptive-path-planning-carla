import carla
import random

# Connect to the client
client = carla.Client('localhost', 2000)
client.set_timeout(10.0)
world = client.get_world()

# Get the blueprint library
blueprint_library = world.get_blueprint_library()

# Choose a vehicle blueprint (e.g., Tesla Model 3)
vehicle_bp = blueprint_library.find('vehicle.tesla.model3')

# Optionally randomize color if the blueprint supports it
if vehicle_bp.has_attribute('color'):
    color = random.choice(vehicle_bp.get_attribute('color').recommended_values)
    vehicle_bp.set_attribute('color', color)

# Get a spawn point (CARLA maps have predefined spawn points)
spawn_points = world.get_map().get_spawn_points()
spawn_point = random.choice(spawn_points)

# Spawn the vehicle
vehicle = world.spawn_actor(vehicle_bp, spawn_point)

print(f"Spawned vehicle: {vehicle.type_id} at {spawn_point.location}")
