"""
carla_utils.py
Shared helper functions for connecting to CARLA and spawning actors.
Import these into your other scripts instead of rewriting connection code each time.
"""

import carla
import random
import math


def connect(host='localhost', port=2000, timeout=10.0):
    """Connect to the CARLA server and return (client, world)."""
    client = carla.Client(host, port)
    client.set_timeout(timeout)
    world = client.get_world()
    return client, world


def spawn_ego_vehicle(world, model='vehicle.tesla.model3'):
    """Spawn a single vehicle at a random spawn point. Returns the vehicle actor."""
    blueprint_library = world.get_blueprint_library()
    vehicle_bp = blueprint_library.find(model)
    spawn_points = world.get_map().get_spawn_points()
    spawn_point = random.choice(spawn_points)
    vehicle = world.try_spawn_actor(vehicle_bp, spawn_point)
    if vehicle:
        print(f"Spawned {vehicle.type_id} at {spawn_point.location}")
    else:
        print("Spawn failed — spawn point may be occupied, try again")
    return vehicle


def move_spectator_to(world, actor, height=30):
    """Teleport the spectator camera to a bird's-eye view above the given actor."""
    spectator = world.get_spectator()
    transform = actor.get_transform()
    spectator.set_transform(carla.Transform(
        transform.location + carla.Location(z=height),
        carla.Rotation(pitch=-90)
    ))


def distance(loc1, loc2):
    """2D distance between two carla.Location objects."""
    return math.sqrt((loc1.x - loc2.x) ** 2 + (loc1.y - loc2.y) ** 2)


def destroy_actors(actor_list):
    """Safely destroy a list of actors."""
    for actor in actor_list:
        if actor is not None and actor.is_alive:
            actor.destroy()
    print(f"Destroyed {len(actor_list)} actors")
