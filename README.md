
# Adaptive Path Planning and Collision Avoidance — CARLA (SIH26037)

Adaptive global A* planning + potential-field local collision avoidance
for autonomous vehicles on unstructured roads, built on CARLA.

## Requirements

- CARLA **0.9.16** server installed locally (this script's `carla` pip
  package version must match your server exactly, or the client won't
  connect).
- Python 3.8–3.10 (match whatever your CARLA install's PythonAPI targets)
- A GPU capable of running the CARLA server (Unreal Engine rendering)

## Setup

1. Install/unzip CARLA 0.9.16 from https://github.com/carla-simulator/carla/releases
2. Clone this repo:
   ```bash
   git clone <this-repo-url>
   cd adaptive-path-planning-carla
   ```
3. Create a virtual environment and install deps:
   ```bash
   python -m venv venv
   source venv/bin/activate      # Windows: venv\Scripts\activate
   pip install -r requirements.txt
   ```
4. Install the CARLA Python API matching your server version. Either:
   ```bash
   pip install carla==0.9.16
   ```
   or, if that wheel isn't on PyPI for your Python version, install the
   one shipped with your server instead:
   ```bash
   pip install <path-to-CARLA>/PythonAPI/carla/dist/carla-0.9.16-<your-platform>.whl
   ```

## Running

1. Start the CARLA server first:
   ```bash
   # Windows
   CarlaUE4.exe
   # Linux
   ./CarlaUE4.sh
   ```
   Wait until the simulator window fully loads before starting the client.

2. In a second terminal (with the venv activated), run:
   ```bash
   python adaptive_path_planning_carla.py --town Town03
   ```

## Useful flags

| Flag | Default | Description |
|---|---|---|
| `--host` | `127.0.0.1` | CARLA server address |
| `--port` | `2000` | CARLA server port |
| `--town` | `Town03` | Map to load |
| `--pedestrians` | `14` | Number of pedestrians spawned |
| `--traffic` | `8` | Number of mixed-traffic vehicles/two-wheelers |
| `--hazards` | `10` | Number of pothole/debris props scattered on the route |
| `--no-hud` | off | Disable the pygame camera + telemetry window, print status to console instead |

Example with a denser, chaotic scene:
```bash
python adaptive_path_planning_carla.py --town Town03 --pedestrians 25 --traffic 15 --hazards 15
```

## Troubleshooting

- **`Could not import carla`** → the `carla` pip package isn't installed,
  or its version doesn't match your server. Check `pip show carla`.
- **Client hangs / times out connecting** → make sure the CARLA server
  window has finished loading before running the script.
- **No route found between spawn point and goal** → try a different
  `--town` (some small maps don't have enough spawn-point spread).
- **pygame window is black / crashes** → run with `--no-hud`, especially
  over remote desktop or SSH without a display. 
