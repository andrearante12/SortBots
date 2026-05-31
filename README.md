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
- **Adding Isaac Sim** for the multi-robot / SLAM / coordination half of the
  project
- Basic warehouse scene setup.
- ROS 2 bridge between Isaac Sim and the rest of the stack 

### CAD & mechanical design
- Custom robot chassis design in CAD (replacing the XLeRobot IKEA-cart base).
- End-effector design exploration 

### Perception & mapping (sim-first)
- Validate VSLAM library choice (cuVSLAM vs. ORB-SLAM3 vs. RTAB-Map) against
  Isaac Sim ground-truth pose. Pick one before fall.
- Stand up nvblox in sim for 3D mapping, verify the output format integrates
  with Nav2.
- Prototype collaborative SLAM in sim — start with centralized map fusion at a
  base station (simpler baseline); distributed peer-to-peer fusion is stretch.

### Navigation (sim-first)
- Nav2 stack configured against a known map in the warehouse scene.
 


## Docs

- [`docs/setup.md`](docs/setup.md) — first-time install (apt, Miniconda, conda env, ReplicaCAD, overlay)
- [`docs/running.md`](docs/running.md) — activating the env, the verify script, the demo catalog
## License

Project code: see `LICENSE` (to be added). XLeRobot submodule: Apache 2.0 (see `third_party/XLeRobot/LICENSE`).
