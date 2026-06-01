# SortBots

Fully autonomous mobile robots performing collaborative
indoor package logistics, built on
[XLeRobot](https://github.com/Vector-Wangel/XLeRobot).

Unlike commercial AMR
systems that rely on extensive per-facility infrastructure,
SortBots targets the opposite operating point: rapid deployment
in unmapped environments, with collaboration as the primary
mechanism for both efficiency and resilience. Robots divide work via decentralized task allocation,
coordinate passage through shared spaces, and dynamically
re-allocate tasks when one is blocked or delayed. Time-to-completion
scales with fleet size, failure of any single robot does not
halt the task, and the coordination layer requires no central server.

## Current Goals 

### Simulation infrastructure

Basic warehouse scene, 2-robot ROS 2 spawn, and the ROS 2 bridge
between Isaac Sim and the rest of the stack.

### CAD & mechanical design
- Custom robot chassis design in CAD (replacing the XLeRobot IKEA-cart base).
- End-effector design exploration 

### Perception & mapping (sim-first)
- Validate VSLAM library choice (cuVSLAM vs. ORB-SLAM3 vs. RTAB-Map)
- Nvblox in sim for 3D mapping, verify the output format integrates
  with Nav2.
- Prototype collaborative SLAM in sim: map fusion, peer to peer communication, decentralized coordination

### Navigation 
- Nav2 stack configured against a known map in the warehouse scene.
- Explore VLA as an alternative manipulation method
 


## Run commands

Open a fresh shell at the repo root. Never activate both simulators in the same shell, and never source ROS 2 in either.

### ManiSkill (arm manipulation track)

```bash
conda activate lerobot
bash scripts/verify_sim.sh         # smoke test: SAPIEN windows for Fetch + XLeRobot
python scripts/run_xle_demo.py     # XLeRobot demo
```

More detail: [`docs/running.md`](docs/running.md).

### Isaac Sim (mobile / multi-robot / ROS 2 / VSLAM track)

```bash
source scripts/activate_isaac.sh

# Phase 1 — headless verify (empty stage, 100 steps)
python scripts/verify_isaac_sim.py

# Phase 2 — URDF → USD pipeline (writes to assets/generated/, gitignored)
python scripts/import_urdf.py \
    --urdf ~/isaacsim/venv/lib/python3.11/site-packages/isaacsim/exts/isaacsim.asset.importer.urdf/data/urdf/robots/carter/urdf/carter.urdf \
    --out  assets/generated/carter.usd \
    --physics-overrides configs/physics_overrides/carter.json

python scripts/import_urdf.py \
    --urdf third_party/XLeRobot/simulation/Maniskill/assets/xlerobot/xlerobot.urdf \
    --out  assets/generated/xlerobot.usd \
    --physics-overrides configs/physics_overrides/xlerobot.json \
    --fix-base

# Phase 2 — regression test (one-time `pip install pytest` into the Isaac venv)
python -m pytest tests/isaac/ -v

# Phase 2 — watch the sim with a visible viewport (needs a display)
python scripts/view_sim.py inspect assets/generated/carter.usd
python scripts/view_sim.py drive carter            # diff-drive forever
python scripts/view_sim.py drive xlerobot          # prismatic forever
```

Everything except `view_sim.py` is headless (`SimulationApp({"headless": True})`) — that's why `verify`, `import_urdf`, and `pytest` never open a window. Use `view_sim.py` when you want to see the robot. Result files for the headless scripts land under `/tmp/isaac_*` because Kit captures stdout during shutdown. `SORTBOTS_FORCE_REIMPORT=1` forces fresh USDs instead of reusing the cached ones.

More detail: [`docs/isaac_sim_setup.md`](docs/isaac_sim_setup.md) (install + Phase 1), [`docs/isaac_sim_phase2.md`](docs/isaac_sim_phase2.md) (Phase 2 pipeline + drive test).

## Docs

- [`docs/setup.md`](docs/setup.md) — ManiSkill first-time install (apt, Miniconda, conda env, ReplicaCAD, overlay)
- [`docs/running.md`](docs/running.md) — activating the ManiSkill env, the verify script, the demo catalog
- [`docs/isaac_sim_setup.md`](docs/isaac_sim_setup.md) — Isaac Sim 5.1 install, activation, and headless verify
- [`docs/isaac_sim_phase2.md`](docs/isaac_sim_phase2.md) — URDF → USD import pipeline, physics-override JSON schema, regression test

## License

Project code: see `LICENSE` (to be added). XLeRobot submodule: Apache 2.0 (see `third_party/XLeRobot/LICENSE`).
