import carla
import random
import time

client = carla.Client('localhost', 2000)
client.set_timeout(10.0)
world = client.get_world()
blueprint_library = world.get_blueprint_library()

vehicle_bp = blueprint_library.find('vehicle.tesla.model3')
spawn_points = world.get_map().get_spawn_points()
vehicle = world.try_spawn_actor(vehicle_bp, random.choice(spawn_points))

# Apply direct control instead of set_autopilot(True)
control = carla.VehicleControl()
control.throttle = 0.5   # 0.0 to 1.0
control.steer = 0.0      # -1.0 (full left) to 1.0 (full right)
control.brake = 0.0

vehicle.apply_control(control)
print("Driving straight for 5 seconds...")
time.sleep(5)

# Now steer right
control.steer = 0.3
vehicle.apply_control(control)
time.sleep(3)

# Stop
control.throttle = 0.0
control.brake = 1.0
vehicle.apply_control(control)

vehicle.destroy()
