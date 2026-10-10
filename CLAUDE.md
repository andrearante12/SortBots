# SortBots — working notes for Claude

Multi-robot warehouse logistics on XLeRobot. Two independent sim tracks:
**Isaac Sim + ROS 2 Jazzy** (mobility, SLAM, Nav2 — the active one) and
**ManiSkill** (arm manipulation, conda env `lerobot`). They share no code and
must never share a shell.

## To run the sim, use the `sim-run` skill

Don't reverse-engineer the launch files. `scripts/sim_ctl.sh` is the entry
point, and the `sim-run` skill has the runbook, exit codes, and failure modes.

```bash
scripts/sim_ctl.sh status        # is anything running? (exit 4 = no session, 3 = no console)
scripts/sim_ctl.sh console start # bring the dashboard up, detached
scripts/sim_ctl.sh start explore_fresh && scripts/sim_ctl.sh wait running --timeout 420
scripts/sim_ctl.sh stop
```

## The real robot (Jetson Orin Nano + RealSense D435)

A third track: the same dashboard, driving hardware. It runs in a **Jazzy
container** (`docker/jetson/`) because JetPack 6 is Ubuntu 22.04/Humble. Never
source the Jetson host's `/opt/ros/humble` for SortBots. `scripts/jetson.sh
console` starts `run_console.sh --platform real` in the container, and `platform:
real` scenarios run `scripts/run_robot.sh` → `sortbots_bringup.launch.py
platform:=real`. Runbook: `docs/jetson.md`.

- The real stack uses `use_sim_time:=false`, RTAB-Map visual odometry, and no IMU
  (a plain D435 has none, so `wait_imu_to_init` must be false). Nav2 is off until
  a base driver exists.
- `/etc/nv_tegra_release` isn't visible inside the container, so
  `SORTBOTS_PLATFORM=real` has to be set there. `scripts/jetson.sh` sets it.
- The container's DDS is localhost-only on purpose, so a desktop sim on the LAN
  can't cross-talk with `/robot_0/*`.

## Layout

This is **not a colcon workspace** — there is no `src/`, no `package.xml`, no
`setup.py`. Everything is loose scripts referenced by path.

| | |
|---|---|
| `nodes/` | rclpy nodes (`explorer.py`, `task_manager.py`, …), run via `python3 <path>` |
| `launch/` | `ros2 launch ./launch/<file>` — by path, not package |
| `webui/` | dashboard (vanilla JS, no build step) + `serve.py` + `session.py` |
| `scripts/` | Isaac Sim entry points, `run_demo.sh`, `run_console.sh`, `sim_ctl.sh` |
| `configs/` | tuning YAML, `scenarios/` presets |
| `maps/` | saved map library — named, committed snapshots of explored maps |
| `splat/` | optional Gaussian-splat reconstruction of saved maps; runs on the WORKSTATION in its own conda env `splat`, never on the Jetson (`scripts/splat.sh`, `docs/splat.md`) |
| `docs/running.md` | the full runbook |

No custom `.msg`/`.srv`: inter-node protocol is `std_msgs/String` carrying JSON.
Node tuning is plain YAML loaded by the node itself, not ROS params. Nodes take
`--robot-id` and prefix topics by f-string, not `PushRosNamespace`.

## Invariants that cost the most time when violated

- **Never `source /opt/ros/...` in a shell that will launch the sim.** Isaac's
  venv and system ROS 2 must stay separate; `run_demo.sh` and
  `activate_isaac.sh` both exit 1 if `AMENT_PREFIX_PATH` is set. Source ROS
  inside a dedicated `bash -c` subshell instead.
- **conda's `python3` leads PATH even in a "clean" shell.** Anything with a
  `#!/usr/bin/env python3` shebang that needs rclpy (rosbridge, `ros2`) breaks
  with `ModuleNotFoundError: rclpy._rclpy_pybind11`. Prepend
  `/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin` in that subshell.
- **`run_demo.sh` exiting 0 means it launched, not that the run ended.** It
  `setsid`s Isaac and the bringup and returns. Liveness is
  `pgrep -f spawn_warehouse.py`, never that pid.
- **Every `/snapshot` request needs `qos_profile=sensor_data`**, including
  ad-hoc `curl`. A RELIABLE image subscriber kills Isaac's image writers at
  around sim-t=100–140 s.
- **Measure robot speed in sim time, not wall clock** — and don't trust one
  real-time factor. It depends almost entirely on how many camera render
  products are in the scene: ~0.47× for a 2-robot run, ~0.33× once a chase cam
  is added, ~0.25× with the full ROS stack alongside. `scripts/bench_sim.sh`
  measures it, and its header carries the table. Rendering is ~85% of the cost
  but rendering LESS OFTEN doesn't help — only fewer render products do.
- **A ROS timer follows `use_sim_time`.** `create_timer` runs off the node
  clock, so enabling sim time silently stretches every control period by the
  real-time factor — a 2 s explorer tick became 11 s of wall clock and looked
  like a hang. Deadlines that stand for distance belong on sim time; the tick
  rate that keeps a node responsive does not (pass an explicit `STEADY_TIME`
  clock, as `nodes/explorer.py` does).
- **A saved map is loaded by COPY, never in place.** RTAB-Map opens its sqlite
  file read-write even under `--localize` (`Mem/IncrementalMemory=false` stops
  it *learning*, not *writing*), so `run_demo.sh` copies any `--map` path under
  `maps/` to `~/.ros/sortbots_<robot_id>.db` first. Don't "simplify" that away —
  without it every demo dirties a tracked git-lfs object. Saving is the mirror
  problem: the grid needs the stack UP, a safe DB copy needs it DOWN, which is
  why `scripts/maps.sh save` is idempotent and can legitimately exit 5
  ("pending"). Use `scripts/sim_ctl.sh stop --save-map NAME` for both halves.
- **Nav2 costmap `map_topic` must be absolute.** A relative one resolves into
  the costmap sub-namespace and silently gets no map.
- **In params YAML under a namespace, use `/**/<node_name>:`** — a bare
  top-level node-name key silently no-ops.

## Testing

Prefer offline. Seconds, no GPU, no display, no ROS:

```bash
node webui/tests/scenarios_test.mjs   # scenarios tab (no fixture needed)
node webui/tests/splat_test.mjs       # 3D panel splat mode, fake worker (no fixture needed)
node webui/tests/dashboard_test.mjs   # dashboard (needs webui/testdata/)
python3 webui/session.py --list       # scenario validation
python3 -m pytest tests/              # pure-python node logic (system python3 — see below)
```

`tests/*_test.py` needs the SYSTEM python3 (`/usr/bin/python3 -m pytest`), not
conda's — conda's base env doesn't have pytest, and this is the same
PATH-ordering gotcha as the rclpy one above, just for a different package.

Never point a browser at a live sim to test the dashboard — record a bag, build
a fixture, replay it offline.

## Multi-user workflow (Jetson shared machine)

This Jetson is shared by multiple team members (andre, gilbert, liam, natalie,
silas). All users access the same `/home/andre/SortBots` directory via symlink.

- **Never work directly on `main`.** Always create a branch:
  `git checkout -b <username>/<feature-name>`
- **Commit often** — uncommitted changes are invisible to git and can be
  overwritten by another user.
- **Pull before branching** — `git pull && git checkout -b <your-branch>`
- **Don't leave uncommitted changes** in shared files. Stash or commit before
  walking away.

### Per-user ROS 2 build directories

The `ros2_ws` workspace at `/home/andre/ros2_ws` uses per-user build artifacts
so builds don't collide. Source `/etc/profile.d/sortbots-env.sh` or start a new
shell — it sets `COLCON_BUILD_BASE`, `COLCON_INSTALL_BASE`, and `COLCON_LOG_BASE`
to `build_$USER`, `install_$USER`, and `log_$USER` respectively.

When building or sourcing the workspace:

```bash
cd ~/ros2_ws
colcon build                    # uses build_$USER/ and install_$USER/ automatically
source install_$USER/setup.bash # source YOUR install space
```

When running `colcon build` on behalf of a user, always verify that
`COLCON_BUILD_BASE` is set (or pass `--build-base build_$USER --install-base
install_$USER` explicitly). Never use the bare `build/` or `install/` dirs.

## Conventions

- Commit messages: short, terse, lowercase, no body. No Co-Authored-By trailer,
  no "Generated with Claude Code".
- **Never attribute Claude anywhere**: commits, PR titles/descriptions, code
  comments, docs. `.claude/settings.json` sets `attribution` off so the tool's
  defaults agree; don't add it back by hand either.
- Comments in this repo carry *rationale* — most non-obvious lines explain the
  failure mode they prevent, often with the date it was diagnosed live. Match
  that when editing; don't strip it.
