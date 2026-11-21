# SortBots

![ci](https://github.com/andrearante12/SortBots/actions/workflows/ci.yml/badge.svg)

Autonomous mobile robots for collaborative indoor logistics, built on
[XLeRobot](https://github.com/Vector-Wangel/XLeRobot): rapid deployment in
unmapped spaces, with decentralized task allocation between robots.

## Pick and place

The real robot picks a package out of its own basket and places it on the chair beside it (sped up).

<img src="docs/media/demos/pick_and_place.gif" alt="Real robot pick and place" width="300">

## Operator dashboard

<img src="docs/media/demos/operator_view_demo.gif" alt="Operator dashboard sim view">

## SLAM mapping (RTAB-Map)

<img src="docs/media/demos/slam_feature_mapping.gif" alt="RTAB-Map feature matching during SLAM">

## Obstacle detection

| Pedestrian | Another robot |
|---|---|
| ![Detect-pedestrian test case](docs/media/demos/detect_pedestrian.gif) | ![Detect-other-robot test case](docs/media/demos/detect_robot.gif) |

## Fleet mesh comms

| Both in range | 2-hop relay | Direct vs. routed |
|---|---|---|
| ![Comms overlay: both robots in range of the gateway](docs/media/demos/comms_in_range.gif) | ![Comms overlay: a 2-hop relay through the other robot](docs/media/demos/comms_relay_hop.gif) | ![Comms overlay: direct link vs. routed relay around obstacles](docs/media/demos/comms_direct_vs_relay.gif) |

## Docs

[Setup](docs/setup.md) · [Running](docs/running.md) · [Architecture](docs/architecture.md) ·
[Testing](docs/testing.md) · [Real robot](docs/real-robot.md) · [Manipulation](docs/manipulation.md) ·
[Arm](docs/arm.md) · [Head camera](docs/head_camera.md) · [Magnet](docs/magnet.md) ·
[Drive](docs/drive.md) · [Comms](docs/commsim.md)

## License

Project code: license to be added. The XLeRobot submodule is Apache 2.0
(`third_party/XLeRobot/LICENSE`).
