"""
attach_camera.py
Spawns an ego vehicle, attaches an RGB camera to it, and saves frames to
./output/ as it drives on autopilot. This is the raw input your perception
module (YOLO / segmentation) will eventually consume.
"""

import time
import os
import carla
from carla_utils import connect, spawn_ego_vehicle, destroy_actors

CAPTURE_SECONDS = 20
OUTPUT_DIR = 'output'


def main():
    os.makedirs(OUTPUT_DIR, exist_ok=True)

    client, world = connect()
    blueprint_library = world.get_blueprint_library()

    vehicle = spawn_ego_vehicle(world)
    if vehicle is None:
        return
    vehicle.set_autopilot(True)

    camera_bp = blueprint_library.find('sensor.camera.rgb')
    camera_bp.set_attribute('image_size_x', '800')
    camera_bp.set_attribute('image_size_y', '600')
    camera_bp.set_attribute('fov', '90')

    # Roughly windshield height, facing forward
    camera_transform = carla.Transform(carla.Location(x=1.5, z=2.4))
    camera = world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle)

    camera.listen(lambda image: image.save_to_disk(f'{OUTPUT_DIR}/{image.frame}.png'))
    print(f"Camera attached. Capturing frames to ./{OUTPUT_DIR}/ for {CAPTURE_SECONDS}s...")

    try:
        time.sleep(CAPTURE_SECONDS)
    except KeyboardInterrupt:
        pass
    finally:
        camera.stop()
        destroy_actors([camera, vehicle])


if __name__ == '__main__':
    main()
