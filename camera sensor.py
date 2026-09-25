import carla
import random
import time

client = carla.Client('localhost', 2000)
client.set_timeout(10.0)
world = client.get_world()
blueprint_library = world.get_blueprint_library()

# Spawn vehicle (same as before)
vehicle_bp = blueprint_library.find('vehicle.tesla.model3')
spawn_points = world.get_map().get_spawn_points()
vehicle = world.try_spawn_actor(vehicle_bp, random.choice(spawn_points))
vehicle.set_autopilot(True)

# Set up RGB camera
camera_bp = blueprint_library.find('sensor.camera.rgb')
camera_bp.set_attribute('image_size_x', '800')
camera_bp.set_attribute('image_size_y', '600')
camera_bp.set_attribute('fov', '90')

# Mount it on the vehicle (windshield-ish position)
camera_transform = carla.Transform(carla.Location(x=1.5, z=2.4))
camera = world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle)

# Save frames to disk so you can see what it's capturing
camera.listen(lambda image: image.save_to_disk(f'output/{image.frame}.png'))

print("Camera attached, capturing frames to ./output/ ...")
time.sleep(20)  # let it run and capture for 20 seconds

camera.stop()
vehicle.destroy()
camera.destroy()
