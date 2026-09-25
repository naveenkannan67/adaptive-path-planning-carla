"""
planning_loop.py
A minimal perceive -> decide -> control loop skeleton:
  - perceive: check distance to nearby vehicles
  - decide:   very crude obstacle check (replace with DWA/RVO/etc.)
  - control:  apply throttle/steer/brake toward a goal waypoint

This is deliberately basic. Your actual project work replaces the "decide"
section with a real local planner (DWA, RVO, MPC, etc.) and the steering
should use proper goal-direction math instead of steer=0.0.
"""

import random
import time
import carla
from carla_utils import connect, spawn_ego_vehicle, distance, destroy_actors

MAX_TICKS = 300
TICK_SLEEP = 0.05
GOAL_RADIUS = 3.0
OBSTACLE_RADIUS = 5.0


def main():
    client, world = connect()
    map_ = world.get_map()

    vehicle = spawn_ego_vehicle(world)
    if vehicle is None:
        return

    spawn_points = map_.get_spawn_points()
    goal_point = random.choice(spawn_points)
    print(f"Goal: {goal_point.location}")

    try:
        for tick in range(MAX_TICKS):
            vehicle_loc = vehicle.get_location()
            dist_to_goal = distance(vehicle_loc, goal_point.location)

            if dist_to_goal < GOAL_RADIUS:
                print(f"Reached goal at tick {tick}")
                break

            # --- PERCEIVE: check nearby vehicles ---
            nearby_vehicles = world.get_actors().filter('vehicle.*')
            obstacle_close = False
            for other in nearby_vehicles:
                if other.id != vehicle.id:
                    if distance(vehicle_loc, other.get_location()) < OBSTACLE_RADIUS:
                        obstacle_close = True
                        break

            # --- DECIDE: replace this block with your real planner ---
            control = carla.VehicleControl()
            if obstacle_close:
                control.throttle = 0.0
                control.brake = 1.0
            else:
                control.throttle = 0.4
                control.steer = 0.0  # TODO: compute steer-toward-goal here

            # --- CONTROL: apply it ---
            vehicle.apply_control(control)
            time.sleep(TICK_SLEEP)

    finally:
        destroy_actors([vehicle])


if __name__ == '__main__':
    main()
