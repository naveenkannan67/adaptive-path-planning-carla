# Adaptive Path Planning -- CARLA demo (SIH26037)

This runs the same planning/avoidance logic as the browser demo, but
inside CARLA against a real map, real vehicle physics, real pedestrians
and traffic. It is meant as your simulation-stage prototype for the
"validated first in simulation" step in the Feasibility slide.

## 1. Prerequisites

- A machine that can run CARLA (Windows/Linux, 8GB+ VRAM recommended).
  Download the compiled server build (not source) from
  https://github.com/carla-simulator/carla/releases -- pick 0.9.15
  unless you have a reason to use another version.
- Python 3.8-3.10 (CARLA's Python client wheels are version-pinned; 3.10
  is safest for 0.9.15).

## 2. Setup

```bash
# 1. Start the CARLA server first, and leave it running
#    Windows: CarlaUE4.exe
#    Linux:   ./CarlaUE4.sh
#    (add -quality-level=Low if your GPU is modest)

# 2. In a separate terminal, install the matching client + deps
pip install -r requirements.txt

# 3. Run the demo
python adaptive_path_planning_carla.py --town Town03
```

If `pip install carla==0.9.15` fails, your server build may be a
different version -- check the server's version string on startup and
match it exactly (`pip install carla==<that version>`).

## 3. What you'll see

- A pygame window with the ego vehicle's forward camera and a live
  telemetry overlay (status, distance, replans, near misses) --
  the same metrics as the browser demo.
- The ego vehicle drives its route while CARLA-controlled pedestrians
  wander the map and CARLA-controlled two-wheelers/cars drive under the
  traffic manager's autopilot.
- Amber debug points drawn just ahead of the vehicle show the upcoming
  slice of the currently active route (visible from the spectator/editor
  view, not the pygame camera).
- If you don't have `pygame` installed, the script still runs -- it
  falls back to printing telemetry to the console every ~2 seconds.

Useful flags:

```bash
python adaptive_path_planning_carla.py --town Town05 --pedestrians 20 --traffic 12 --hazards 15
python adaptive_path_planning_carla.py --no-hud          # headless, console telemetry only
```

## 4. How this maps to the proposal, and what's simplified

| Proposal item | This script |
|---|---|
| Sensor fusion (LiDAR + camera + radar) | LiDAR, RGB camera and radar are attached and streaming (`attach_sensors`), but the planner currently reads **ground-truth actor poses** from `world.get_actors()` rather than processing the point clouds. This is the standard order of operations -- get planning + control working first, then swap in perception. The sensor callbacks are already wired up as the place to plug that in. |
| Hybrid A* / RL planner | Implemented as A* over a graph built from CARLA's OpenDRIVE topology (`WaypointGraphPlanner`), which is close to what CARLA's own `GlobalRoutePlanner` does. It is **not** the continuous-curvature Hybrid-A* described in the proposal yet -- see "Next steps" below. |
| MPC controller + potential-field safety layer | The local layer (`LocalController`) blends a potential-field repulsion vector with the path-following target, converts that into steering via a heading-error controller and speed via a simple proportional throttle/brake -- this is a lightweight stand-in for the full MPC. A hard-brake safety layer overrides everything if any actor enters `COLLISION_RADIUS`. |
| Potholes / unstructured obstacles | CARLA has no native pothole asset, so `HAZARD_PROP_BLUEPRINTS` scatters debris/traffic-cone props as visual and cost-map stand-ins (they add soft cost to nearby graph edges, same idea as the pothole soft cost in the browser demo). |
| Mixed traffic (two-wheelers, pedestrians) | Real CARLA pedestrians (`walker.pedestrian.*`) with AI walker controllers on randomised destinations, and real two-wheeler/car blueprints under the traffic manager's autopilot. |
| Dynamic risk-based collision buffers | `SENSOR_RADIUS` / `NEAR_MISS_RADIUS` / `COLLISION_RADIUS` thresholds, same three-tier structure as the browser demo. |

## 5. Next steps if you want to push this further

1. **Real Hybrid-A***: replace `WaypointGraphPlanner.route`'s grid/graph
   search with a state-lattice expansion (x, y, heading) and a
   Reeds-Shepp or car-kinematic successor model -- this is the piece
   that would make the "Hybrid A*" claim in the deck fully literal
   rather than approximate.
2. **Real perception**: cluster the LiDAR point cloud (already
   streaming) into obstacle candidates instead of using
   `world.get_actors()`, so the local controller reacts to what the
   vehicle actually "senses" rather than ground truth. That is the
   natural next milestone and demonstrates the sensor-fusion claim.
3. **MPC**: swap the proportional steering/speed controller for a real
   receding-horizon MPC (e.g. via `cvxpy` or `do-mpc`) if you want the
   controller to literally match the proposal rather than approximate it.
4. **Indian-road texture**: CARLA's default towns are structured
   (lane-marked) roads. For a more honest "unstructured Indian road"
   scenario, either use a CARLA map pack with unmarked rural roads, or
   accept that this demo shows the *decision logic* working under mixed
   dynamic traffic, and pair it with the India Driving Dataset (IDD)
   references already in your research slide for the perception side.

## 6. Known simplifications / things to watch for

- Obstacle avoidance uses actor ground truth, not sensed geometry (see
  above) -- call this out explicitly if asked in Q&A, don't imply it's
  full sensor fusion.
- The goal point is chosen as the spawn point farthest from the ego's
  start, which is a reasonable default for a demo but not meaningful
  route selection -- change `goal_point` in `main()` if you want a
  fixed, repeatable start/goal pair for judging.
- This was authored without a live CARLA server to test against (no
  GPU/display in the authoring environment), so treat this as a strong
  first draft: run it, and if you hit an API mismatch (CARLA's Python
  API has shifted slightly across 0.9.x versions), paste me the error
  and I'll fix it directly.
