# Task: Add Isaac Sim as a second simulator alongside ManiSkill

## Project context

This is the SortBots capstone — a 5–6 person team building 2 autonomous warehouse robots on top of the XLeRobot platform (https://xlerobot.readthedocs.io/). The current simulator stack is ManiSkill (SAPIEN-based), which ships with XLeRobot and is great for arm manipulation / VLA work but does not support multi-robot mobile navigation, ROS 2, or VSLAM.

We are **adding Isaac Sim** for the mobile-robot / multi-robot / SLAM / coordination half of the project. ManiSkill stays for VLA. However we need to do further research into if we can fully pivot into Issac Sim for convenience.

## Why Isaac Sim specifically (don't re-litigate this)

- Native ROS 2 bridge — our hardware stack is ROS 2
- Same ecosystem as Isaac ROS / cuVSLAM, which runs on the Jetson Orin Nano on each robot
- Same VSLAM code and ROS topics run in sim and on hardware → sim-to-real for perception is nearly free
- GPU-parallel envs for MARL training (stretch goal)

## Target environment

- Workstation: Ubuntu 24.04 LTS, 4070 NVIDIA RTX GPU (≥8 GB VRAM, RT cores required), 32+ GB RAM
- Isaac Sim version: 5.x (latest production)
- Python 3.11 (Isaac Sim 5.x requirement — do not assume 3.10)
- ROS 2 Jazzy
- Jetson Orin Nano on robot side runs Isaac ROS, not Isaac Sim — do not try to install sim on Jetson

## Repo state to read first

Before doing anything, read these files in the repo (run `ls` and `find` to locate them if paths differ):
- `README.md`
- Any `pyproject.toml`, `requirements.txt`, or `environment.yml`
- The existing ManiSkill setup files (likely under `simulation/` or `sim/`)
- Any URDF files we've already produced
- Any ROS 2 workspace structure (look for `src/`, `package.xml`, `CMakeLists.txt`)

Summarize what you find before proposing changes. **Do not modify anything ManiSkill-related.**

## Deliverables (in order)

### Phase 1 — Install & verify (do this first, stop and confirm before Phase 2)
1. Generate an install script (`scripts/install_isaac_sim.sh`) that:
   - Checks for required NVIDIA driver version (R525+ Linux)
   - Checks GPU has RT cores (fail clearly if not)
   - Downloads and installs Isaac Sim 5.x via the official method
   - Sets up Python 3.11 venv specifically for Isaac Sim work, separate from any existing ManiSkill env
2. Generate a `scripts/verify_isaac_sim.py` that launches Isaac Sim headlessly, loads a default scene, steps the simulator 100 times, and exits cleanly. This is the "did install actually work" check.
3. Write `docs/isaac_sim_setup.md` explaining install, troubleshooting, and how to run the verify script.

**STOP after Phase 1.** Report back with the install script, what you verified, and any issues. Wait for me to confirm the install worked on the target workstation before continuing.

### Phase 2 — URDF import pipeline
1. Build `scripts/import_urdf.py` that takes a URDF path and produces a USD asset suitable for Isaac Sim:
   - Imports with `Fix Base Link = false`, `Self Collision = true` (per BEHAVIOR tutorial conventions for mobile robots)
   - Converts mesh wheel colliders to cylinder primitives (PhysX GPU pipeline performance — known wheel gotcha)
   - Applies a configurable physics-override JSON for joint stiffness/damping/friction (so we don't lose these tweaks every re-import)
2. Use a placeholder URDF for now (any simple mobile-robot URDF — TurtleBot3 or the Carter robot ship with Isaac Sim). Confirm import works end-to-end with the placeholder before integrating our team's real URDF (which is still being designed).
3. Write a regression test that imports the placeholder URDF, spawns it in a flat-ground scene, drives it forward for 5 seconds, and asserts the robot moves >1 meter without exploding.

### Phase 3 — Warehouse scene + 2-robot spawn
1. Build a basic warehouse scene under `scenes/warehouse_v0.usd`:
   - Floor, walls, 4–6 static obstacles (boxes / shelves)
   - Lighting suitable for camera rendering
   - Defined pickup and dropoff zones (named USD prims so we can reference them programmatically)
2. Script that spawns 2 instances of the imported robot at distinct start positions
3. Each robot publishes its odometry and one RGB-D camera stream over ROS 2 (use the Isaac Sim ROS 2 Bridge, not custom code)
4. Verify with `ros2 topic list` and `ros2 topic echo` that both robots' topics are live and distinct (e.g., `/robot_0/odom`, `/robot_1/odom`, `/robot_0/camera/depth`, etc.)

### Phase 4 — cuVSLAM hook-up (stretch within this task)
1. Document (don't necessarily run) how to point Isaac ROS cuVSLAM at the simulated camera streams
2. Provide a launch file template

## Hard constraints

- **Do not touch ManiSkill files or environments.** Keep envs separate.
- **Ask before installing system packages with sudo.** Show me the command first.
- **Ask before downloading anything over 1 GB.** Isaac Sim itself is large; that's expected, but flag it.
- If you hit a URDF import error like "static 3D model, joints disabled" — that's a known Isaac Sim 5.0 issue. Don't spend more than 30 min on it before asking; there are documented import-flag fixes.
- Python 3.11 specifically. Isaac Sim 5.x is built against 3.11; using 3.10 will silently break extensions.

## What "done" looks like for this task

- `./scripts/install_isaac_sim.sh` runs cleanly on a fresh Ubuntu 22.04 box with an RTX GPU
- `./scripts/verify_isaac_sim.py` exits 0
- `./scripts/import_urdf.py path/to/turtlebot3.urdf` produces a working USD
- Running the warehouse launch script spawns 2 robots and publishes distinct ROS 2 topics for each
- Docs in `docs/isaac_sim_setup.md` are clear enough that another team member can replicate the setup

## Out of scope (do not do)

- MARL training infrastructure
- Custom Nav2 configuration (that's WS6's job)
- Importing the team's actual robot URDF (it doesn't exist yet — placeholder only)
- Anything Jetson-side
- Touching ManiSkill or VLA code

Start with Phase 1 and stop for confirmation.