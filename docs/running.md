# Running the sim

All commands below assume `conda activate lerobot` and `cd ~/sortbots_ws`.

## The verify script

```bash
bash scripts/verify_sim.sh
```

Runs three things in order:

1. **Sanity imports + headless rollout** — imports `mani_skill`, `torch`, `Xlerobot`, builds a `ReplicaCAD_SceneManipulation-v1` env with the xlerobot agent, resets, and steps 5 zero-actions. Fails fast (before any window opens) if the overlay is broken or the conda env is misconfigured.
2. **Demo 1** — `demo_random_action` on Fetch in ReplicaCAD, `rt-fast` shader.
3. **Demo 2** — XLeRobot dual-arm in ReplicaCAD via `scripts/run_xle_demo.py`, `default` shader.

Close each SAPIEN viewer window to advance to the next step.

## The XLeRobot demo launcher

ManiSkill's bundled `demo_random_action` works with `-r xlerobot` but only steps random actions. For richer control (keyboard, gamepad, Rerun visualization), use XLeRobot's own example scripts under `third_party/XLeRobot/simulation/Maniskill/examples/`. They don't import `mani_skill.envs` themselves, so a thin shim is required to populate gym's env registry before they run. That shim is `scripts/run_xle_demo.py`:

```bash
python scripts/run_xle_demo.py <demo_name> [demo args...]
# List all available demos:
python scripts/run_xle_demo.py --help
```

`<demo_name>` is the file stem of any `.py` under the XLeRobot examples directory.

## Demo catalog

All examples below use the ReplicaCAD apartment scene. Substitute another mani-skill env id (e.g. `PushCube-v1`) for tabletop tasks.

### Preset action stepper (what verify_sim.sh runs as Demo 2)

```bash
python scripts/run_xle_demo.py demo_ctrl_action \
    -e "ReplicaCAD_SceneManipulation-v1" \
    -r "xlerobot" \
    --render-mode="human" \
    --shader="default" \
    -c "pd_joint_delta_pos_dual_arm"
```

### Keyboard end-effector teleop, dual-arm

```bash
python scripts/run_xle_demo.py demo_ctrl_action_ee_keyboard \
    -e "ReplicaCAD_SceneManipulation-v1" \
    -r "xlerobot" \
    --render-mode="human" \
    --shader="default" \
    -c "pd_joint_delta_pos_dual_arm"
```

Keymap prints on launch (WASD for the mobile base, plus per-arm IK targets).

### Keyboard end-effector teleop, single-arm

```bash
python scripts/run_xle_demo.py demo_ctrl_action_ee_keyboard_single \
    -e "ReplicaCAD_SceneManipulation-v1" \
    -r "xlerobot_single" \
    --render-mode="human" \
    --shader="default" \
    -c "pd_joint_delta_pos"
```

### Xbox / Switch / Bluetooth controller teleop

```bash
python scripts/run_xle_demo.py demo_ctrl_action_ee_xbox \
    -e "ReplicaCAD_SceneManipulation-v1" \
    -r "xlerobot" \
    --render-mode="human" --shader="default" \
    -c "pd_joint_delta_pos_dual_arm"
```

Sanity-check the controller bindings first with `python scripts/run_xle_demo.py test_xbox`.

### Camera streams visualized with Rerun

```bash
python scripts/run_xle_demo.py demo_ctrl_action_ee_cam_rerun \
    -e "ReplicaCAD_SceneManipulation-v1" \
    -r "xlerobot" \
    --render-mode="human" --shader="default" \
    -c "pd_joint_delta_pos_dual_arm"
```

Opens a Rerun viewer alongside the SAPIEN window with per-camera streams.

### Record a teleop dataset

```bash
python scripts/run_xle_demo.py demo_ctrl_ee_keyboard_record_dataset \
    -e "ReplicaCAD_SceneManipulation-v1" \
    -r "xlerobot" \
    --render-mode="human" --shader="default" \
    -c "pd_joint_delta_pos_dual_arm"
```

Writes episodes to the path the script prints. Useful as input to LeRobot imitation-learning pipelines.

### VR teleop (Quest 3)

```bash
python scripts/run_xle_demo.py demo_ctrl_action_ee_VR \
    -e "ReplicaCAD_SceneManipulation-v1" \
    -r "xlerobot" \
    --render-mode="human" --shader="default" \
    -c "pd_joint_delta_pos_dual_arm"
```

Requires a Quest 3 reachable from the host; see the XLeRobot docs for the pairing flow.

## Supported control modes for xlerobot

Pass with `-c`. From `Xlerobot.supported_control_modes`:

- Dual-arm: `pd_joint_pos_dual_arm`, `pd_joint_delta_pos_dual_arm`
- Single primary arm: `pd_joint_pos`, `pd_joint_delta_pos`, `pd_joint_target_delta_pos`, `pd_joint_vel`, `pd_joint_pos_vel`, `pd_joint_delta_pos_vel`, `pd_joint_delta_pos_stiff_body`
- Second arm (suffixed `_arm2`): same set as above with `_arm2` appended

Use `xlerobot_single` as `-r` when running the single-arm variant of XLeRobot.

## Headless usage

For automation / CI / RL rollouts, set `--render-mode="rgb_array"` (returns batched images via `env.render()`) or `--render-mode="sensors"` (only the agent's onboard cameras). The headless smoke test inside `verify_sim.sh` shows the pattern.

## Bumping the XLeRobot submodule

```bash
git submodule update --remote third_party/XLeRobot
python scripts/overlay_xlerobot.py    # re-apply overlay against the new submodule
git add third_party/XLeRobot && git commit -m "bump XLeRobot"
```

See [`overlay.md`](overlay.md) for what the overlay step does and when it might need adjustment.

## The full SLAM + Nav demo (web dashboard)

The end-to-end warehouse demo — Isaac Sim, RTAB-Map SLAM, Nav2, and the web
dashboard — comes up with one command from a **clean** terminal (no ROS
sourced, no Isaac/conda active):

```bash
scripts/run_demo.sh
```

It launches Isaac Sim in its own venv, then the whole ROS 2 side in a single
launch (`launch/sortbots_bringup.launch.py`), and prints the dashboard URL.
rviz is off by default (the dashboard replaces it); add `--teleop` for the old
WASD terminal window. Tear it all down with `scripts/run_demo.sh stop`.

### Unified ROS-2 bringup on its own

If Isaac Sim is already running (e.g. you launched `scripts/spawn_warehouse.py`
by hand), bring up just the ROS 2 side from a shell with system ROS 2 Jazzy
sourced (and `which python3` == `/usr/bin/python3` — see the webui launch
docstring for why conda's Python breaks rosbridge):

```bash
source /opt/ros/jazzy/setup.bash
ros2 launch ./launch/sortbots_bringup.launch.py
```

This includes RTAB-Map SLAM + Nav2 + the dashboard stack (rosbridge 9090 +
web_video_server 8080 + serve.py 8081) + `nodes/task_manager.py`. Toggle
pieces with `nav2:=false`, `webui:=false`, `task_manager:=false`, `rviz:=true`.
Startup order is tolerant: Nav2 autostarts and retries until RTAB-Map's map/TF
appear.

### What the dashboard shows

Open the printed URL (`http://localhost:8081/` locally, or the tailnet URL).
The page is sized to fit one viewport — nothing scrolls except the task queue.

**The stage** (the big box on the left) shows one view at a time, with a second
one as a picture-in-picture inset:

- **Chase cam** (default) large, **head cam** as the inset. **Click the inset to
  swap** which feed is large.
- **Map mode** — the `camera` / `map` buttons in the header swap the map into
  the stage in place of the cameras; the chase cam stays as the inset so you
  don't lose sight of the robot while picking a goal. Switching back restores
  whichever camera you last had large.
- **Drive pad** (bottom-left) and **head-aim pad** (bottom-right) float over the
  stage as translucent overlays — they fade up on hover, or on tap on a phone.
  The aim pad is hidden in map mode, where head pan/tilt means nothing.

A feed that's off the stage stops being fetched, so map mode isn't paying for
the head camera in the background.

**The right column** is always visible: the **3D reconstruction** viewer on top,
**dispatch + task queue** below.

- **Map view** — the RTAB-Map occupancy grid, the robot's cyan
  odom trail, plus live Nav2 overlays:
  - **Planned path** (orange) — Nav2's current global plan, for both
    click-to-nav and dispatched pickup→dropoff tasks.
  - **Global costmap** (translucent red tint) — toggle with the "costmap"
    checkbox; shows inflation/lethal cells around obstacles.
  - **Goal marker** (orange ring + heading tick) — the current nav goal,
    derived from the planned path's endpoint so it's correct regardless of who
    set the goal.
- **SLAM status badge** (header) — "mapping", flashes "loop closed" and counts
  total loop closures from `/robot_0/info`; goes red/"stale" if RTAB-Map stops
  publishing.

### Driving

W/A/S/D to move, Q/E to rotate, Space to stop — the keys work anywhere on the
page, no need to click the pad first (typing in the dispatch selects doesn't
drive the robot, and switching tabs mid-drive stops it). The pad buttons also
work by holding the mouse down, or by touch on a phone. This publishes straight
to `cmd_vel`, so don't drive while a task is active — Nav2 will fight you.

### Click-to-navigate

Switch the stage to **map**, then drag on it (rviz "2D Nav Goal" style):
**press** sets the goal position, **drag** sets the heading (a plain click uses
heading 0). The goal that was sent is echoed bottom-right. This sends a
`NavigateToPose` goal straight to Nav2. **Caveat:** a manual map goal preempts
whatever `task_manager` is currently navigating — it's a manual override, same
spirit as the drive-pad note above. Use the **Dispatch task** form for the
queued pickup→dropoff workflow instead.

## Remote access from a phone / remote Claude session (Tailscale)

The web dashboard (`launch/sortbots_webui.launch.py` → `webui/serve.py` on 8081,
`rosbridge_websocket` on 9090, `web_video_server` on 8080) all bind `0.0.0.0`, so
any device on the Tailscale tailnet can drive it — a phone for remote testing, or
a remote Claude session that brings the UI up on this workstation.

**Prerequisites**

- `tailscale up` on both the workstation and the phone, on the same tailnet.
- No firewall change needed on the default desktop (`ufw` is disabled here). If
  you enable `ufw`, add `sudo ufw allow in on tailscale0`.

**Bring it up** (from a shell with system ROS 2 Jazzy sourced — *not* the Isaac
venv or conda; `which python3` **must** print `/usr/bin/python3`, or
`rosbridge_websocket`'s `#!/usr/bin/env python3` shebang grabs conda's Python and
crashes with `No module named 'rclpy._rclpy_pybind11'` — see the launch file's
docstring):

```bash
source /opt/ros/jazzy/setup.bash
which python3   # must be /usr/bin/python3
ros2 launch ./launch/sortbots_webui.launch.py
```

On startup it prints the phone URL (and a QR if `pip install qrcode` is present):

```
http://andre-ubuntu.tail0d28f9.ts.net:8081/     (IP fallback http://100.74.199.76:8081/)
```

Open that on the phone. The page is served over plain HTTP on purpose:
`webui/app.js` derives the rosbridge (`ws://…:9090`) and video (`http://…:8080`)
hosts from `window.location.hostname`, so an HTTP origin lets them auto-wire to
the same host. An HTTPS origin (e.g. Tailscale Serve) would make those
mixed-content and the browser would block them — plain HTTP on the private
tailnet avoids that with no code changes. `roslib` is vendored at
`webui/vendor/roslib.min.js`, so the page needs no public internet to load.

Print the URL again any time without relaunching:

```bash
python3 scripts/webui_url.py
```

**Troubleshooting** (from the workstation)

```bash
ss -tlnp | grep -E ':8080|:8081|:9090'   # all three listening on 0.0.0.0?
tailscale status                          # phone + workstation both online?
ufw status                                # inactive, or tailscale0 allowed?
curl -s -o /dev/null -w '%{http_code}\n' http://<tailnet-ip>:8081/   # expect 200
```

## Testing the dashboard without the sim (recorded data)

Checking a dashboard change shouldn't need Isaac Sim, a GPU and a display. So
record real ROS data **once**, and replay it from then on:

```
scripts/record_dashboard_bag.sh          # ← the ONLY step that needs the sim
python3 scripts/bag_to_fixture.py <bag>  # offline
node webui/tests/dashboard_test.mjs      # offline, no ROS at all, seconds
scripts/replay_dashboard_bag.sh <bag>    # offline, real rosbridge + browser
```

### 1. Record (needs the demo running)

With `scripts/run_demo.sh` up and ROS 2 sourced:

```bash
scripts/record_dashboard_bag.sh --duration 60
```

**Drive the robot while it records, and dispatch a task.** A bag of a
stationary robot has no trail, no planned path, no task states and probably no
loop closure, so the fixture can't exercise those panels. The script warns
about any dashboard topic that isn't currently being published before it starts
— read that list, it's the difference between a useful bag and a useless one.

Bags land in `data/bags/` (gitignored, they're hundreds of MB).

### 2. Build the fixture

```bash
python3 scripts/bag_to_fixture.py data/bags/dashboard_<timestamp>/
```

This trims the bag to a small committable snapshot in `webui/testdata/`:
`fixture.json` plus a handful of JPEG camera frames. It prints a per-topic
kept/dropped table and fails if the result exceeds 5 MB.

Messages are serialised with rosbridge's *own* converter
(`rosbridge_library.internal.message_conversion.extract_values`), so the fixture
is byte-shaped exactly like what the browser receives live — including
base64 for `uint8[]` fields but plain arrays for `OccupancyGrid.data`.

### 3. Run the headless test

```bash
node webui/tests/dashboard_test.mjs                  # add --screenshot DIR for PNGs
```

No ROS, no sim, no display, no npm install. It serves the real page, swaps in a
stubbed `roslib` and fake `web_video_server`, replays the fixture, and asserts
across five viewports: layout fits without scrolling, PiP swap, map mode, the
grid renders undistorted, click-to-nav round-trips to the right world
coordinate, the trail and robot marker draw, keyboard driving and its form
guard, and the polling gate. Exit code is non-zero on any failure.

### 4. Replay through the real stack (optional)

To eyeball it in a real browser, with real rosbridge and real MJPEG:

```bash
scripts/replay_dashboard_bag.sh data/bags/dashboard_<timestamp>/
# then open http://localhost:8081/
```

This is also how to double-check fixture fidelity: what the browser gets here
went through the live rosbridge serializer, which is the same `extract_values`
the fixture was built with.
