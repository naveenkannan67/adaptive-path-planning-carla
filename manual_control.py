"""
manual_control.py
Drives the ego vehicle with direct throttle/steer/brake commands instead of
autopilot. This is the control interface (carla.VehicleControl) your planner
will eventually drive programmatically instead of these hardcoded values.
"""

import time
import carla
from carla_utils import connect, spawn_ego_vehicle, destroy_actors


def main():
    client, world = connect()
    vehicle = spawn_ego_vehicle(world)
    if vehicle is None:
        return

    control = carla.VehicleControl()

    try:
        # Drive straight
        control.throttle = 0.5
        control.steer = 0.0
        control.brake = 0.0
        vehicle.apply_control(control)
        print("Driving straight...")
        time.sleep(5)

        # Steer right
        control.steer = 0.3
        vehicle.apply_control(control)
        print("Steering right...")
        time.sleep(3)

        # Stop
        control.throttle = 0.0
        control.steer = 0.0
        control.brake = 1.0
        vehicle.apply_control(control)
        print("Braking...")
        time.sleep(2)

    finally:
        destroy_actors([vehicle])


if __name__ == '__main__':
    main()
