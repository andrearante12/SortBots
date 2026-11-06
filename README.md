# SortBots

![ci](https://github.com/andrearante12/SortBots/actions/workflows/ci.yml/badge.svg)

![Warehouse simulation environment](docs/media/warehouse_hero.png)

Fully autonomous mobile robots performing collaborative
indoor logistics, built on
[XLeRobot](https://github.com/Vector-Wangel/XLeRobot).

Unlike commercial AMR
systems that rely on extensive per-facility infrastructure,
SortBots targets the opposite operating point: rapid deployment
in unmapped environments, with collaboration as the primary
mechanism for both efficiency and resilience. Robots divide work via decentralized task allocation,
coordinate passage through shared spaces, and dynamically
re-allocate tasks when one is blocked or delayed. Applications to autonomous delivery logistics in dynamic enviornemnts, such as warehouses, shelf restocking, or serving foods in restaurants.

### Operator dashboard

Driving, teleport, waypoints and live camera/depth feeds for both robots from one browser tab:

![Operator dashboard sim view](docs/media/demos/operator_view_demo.gif)

### SLAM Mapping and 3D Reconstruction (RTAB-Map) 

![RTAB-Map feature matching during SLAM](docs/media/demos/slam_feature_mapping.gif)

### Object detection validation

Scripted test cases from the [simtest](simtest/) layer: detecting a crossing pedestrian, and one robot detecting another.

| Detect pedestrian | Detect other robot |
|---|---|
| ![Detect-pedestrian test case](docs/media/demos/detect_pedestrian.gif) | ![Detect-other-robot test case](docs/media/demos/detect_robot.gif) |

### Fleet mesh comms overlay

The Comms overlay modeling Wi-Fi mesh range and link quality between robots and the gateway: both robots in direct range of the gateway, a robot out of range relaying through the other, and the overlay comparing a direct line-of-sight link against the routed path when obstacles are in the way.

| Both robots in range | 2-hop relay | Direct vs. routed |
|---|---|---|
| ![Comms overlay: both robots in range of the gateway](docs/media/demos/comms_in_range.gif) | ![Comms overlay: a 2-hop relay through the other robot](docs/media/demos/comms_relay_hop.gif) | ![Comms overlay: direct link vs. routed relay around obstacles](docs/media/demos/comms_direct_vs_relay.gif) |

### Simulation infrastructure
- **Isaac Sim** Isaac Sim 5.1 supporting SLAM pipelines and integrating with Nvidia packages such as RTAB-Map

## Quickstart

Ubuntu 22.04 or 24.04 on X11, an NVIDIA RTX GPU with at least 8 GB of VRAM. Install steps
are in [docs/setup.md](docs/setup.md). Start from a clean shell, and never source ROS 2 in it:

```bash
scripts/run_console.sh                  # dashboard at http://localhost:8081/ (leave running)
# Session → explore_fleet → Start: Isaac Sim + RTAB-Map + Nav2, two robots exploring
scripts/run_console.sh stop             # console and any running sim
```

The same, scripted (exit codes are the interface):

```bash
scripts/sim_ctl.sh console start
scripts/sim_ctl.sh start explore_fleet headless=true
scripts/sim_ctl.sh wait running --timeout 420
scripts/sim_ctl.sh stop
```

## Documentation

| Doc | Covers |
|---|---|
| [docs/setup.md](docs/setup.md) | requirements, installing Isaac Sim and ROS 2, troubleshooting |
| [docs/running.md](docs/running.md) | console, `sim_ctl.sh`, `run_demo.sh`, scenarios, performance, remote access, the operator view, saved maps |
| [docs/architecture.md](docs/architecture.md) | repo layout, the invariants that cost the most time, perception, Nav2 and exploration, the fleet auction |
| [docs/testing.md](docs/testing.md) | offline checks, scored live tests (worlds, test cases, metrics) |
| [docs/real-robot.md](docs/real-robot.md) | the Jetson + D435, SLAM offload, Gaussian splats |
| [docs/manipulation.md](docs/manipulation.md) | magnet pick and place in the sim, the magnet end effector CAD, the ManiSkill arm track |
| [docs/commsim.md](docs/commsim.md) | fleet radio mesh and task-protocol simulation, real routers |

Notes for coding agents are in `CLAUDE.md` and the `sim-run` skill (`.claude/skills/sim-run/`).
Some code comments cite older design notes and run logs as `path@6501799`. Those files were
folded into these docs, and the originals can be read with `git show 6501799:path`.

## License

Project code: license to be added. The XLeRobot submodule is Apache 2.0
(`third_party/XLeRobot/LICENSE`).
