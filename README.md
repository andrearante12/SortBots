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
 


## Docs

- [`docs/setup.md`](docs/setup.md) — ManiSkill first-time install (apt, Miniconda, conda env, ReplicaCAD, overlay)
- [`docs/running.md`](docs/running.md) — activating the ManiSkill env, the verify script, the demo catalog
- [`docs/isaac_sim_setup.md`](docs/isaac_sim_setup.md) — Isaac Sim 5.1 install, activation, and headless verify
## License

Project code: see `LICENSE` (to be added). XLeRobot submodule: Apache 2.0 (see `third_party/XLeRobot/LICENSE`).
