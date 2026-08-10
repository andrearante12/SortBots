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

### Console mode: launching scenarios from the dashboard

`run_demo.sh` stays the CLI path and is unchanged. But you can also start and
stop the sim **from the dashboard's Scenarios tab**, which is the nicer way to
work through a series of runs (and the only way to do it from your phone).

That needs one structural change, because the dashboard used to be *inside* the
pipeline: the bringup started rosbridge/web_video_server/`serve.py`, and
`run_demo.sh stop` killed them. A "start the sim" button can't live in a process
the sim owns — the first click would kill the page mid-request. So console mode
splits the dashboard out into a process that outlives sim runs:

```bash
scripts/run_console.sh          # clean terminal, no ROS sourced. Leave it running.
```

That starts only rosbridge (9090), web_video_server (8080) and
`webui/serve.py --control` (8081), prints the dashboard URL, and stays in the
foreground. Open the dashboard, switch the header to **scenarios**, pick a card
and hit **Start**: `webui/session.py` runs `scripts/run_demo.sh` for you with
`--keep-console`, which makes its teardown spare the console processes and
makes the bringup start with `webui:=false` (two rosbridges would fight over
port 9090). **Stop** tears the sim down and leaves the console — and your open
page — up.

- Ctrl-C in the console terminal stops the console only; a running sim keeps
  going. `scripts/run_console.sh stop` takes down both.
- Each run gets `data/sessions/<timestamp>_<scenario>/` with `console.log`
  (what the tab tails) and `session.json`.
- The console can be restarted mid-run: on startup it re-adopts a live pipeline
  rather than claiming nothing is running.
- Without a console, `serve.py` runs read-only and the tab says so instead of
  offering buttons that can't work.

#### Driving it from a script (or a coding agent)

Everything the tab does is also one command. `scripts/sim_ctl.sh` wraps the
control API so nothing has to hand-write JSON or reason about foreground vs
background:

```bash
scripts/sim_ctl.sh console start                  # detached; waits until :8081 answers
scripts/sim_ctl.sh list                           # scenarios (works with no console up)
scripts/sim_ctl.sh dry-run explore_fresh headless=true   # print the command, launch nothing
scripts/sim_ctl.sh start explore_fresh headless=true
scripts/sim_ctl.sh wait running --timeout 420     # first run streams NVIDIA assets
scripts/sim_ctl.sh status                         # state, phase, scenario, elapsed
scripts/sim_ctl.sh log --lines 100
scripts/sim_ctl.sh stop                           # sim down, console stays up
```

Exit codes are the interface — branch on them, not on the text: `0` success,
`1` command failure (bad scenario, rejected start, run went to `failed`), `3`
no console, `4` no session (which is how you ask "is anything running?"), `124`
timed out. `wait` dumps the last 30 log lines when a run fails.

Coding agents should use the **`sim-run` skill** (`.claude/skills/sim-run/`),
which carries this runbook plus the failure modes; `CLAUDE.md` points at it.

The phase readout (`starting Isaac Sim → loading the warehouse → sim publishing
→ starting RTAB-Map + Nav2 → ROS 2 stack up → running`) is parsed out of
`run_demo.sh`'s own progress lines by `PHASE_PATTERNS` in `webui/session.py`.
If you reword those echoes, update that table in the same commit.

> **The control API is not authenticated.** `serve.py` binds `0.0.0.0`, so with
> `--control` any tailnet device can launch a scenario. Containment is that
> scenario names resolve by allowlist against `configs/scenarios/`, and every
> argument handed to `run_demo.sh` is built by the fixed, typed `RUN_FLAGS`
> table in `webui/session.py` — no free-text value from a request reaches a
> shell. Tailscale ACLs are the actual perimeter.

#### Adding a scenario

A scenario is a reviewable preset for `run_demo.sh`, one YAML per file in
`configs/scenarios/` (the filename stem must equal `name`):

```yaml
name: explore_fresh
title: Frontier exploration — fresh map
description: >
  Shown on the card. Say what the run does and what it proves.
status: ready                 # ready | planned (planned lists but can't start)
run:                          # allowlisted keys only — see RUN_FLAGS
  scene: nvidia               # nvidia | primitive
  robots: 1                   # 1..2
  robot_id: robot_0
  headless: false
  chase_cam: true             # false -> --no-chase-cam
  chase_cam_robots: null      # null = every spawned robot; int = first N only
  explore: true
  resume: false               # resume and localize are mutually exclusive
  localize: false
  teleop: false
  map: null                   # null -> ~/.ros/sortbots_<robot_id>.db
overrides: [headless, robots] # which keys the card exposes as controls
capture:
  bag: false                  # true also records scripts/record_explore_bag.sh
```

Unknown `run:` keys are rejected, not forwarded — the card renders with status
`invalid` and the reason, which is also what you get from the offline check:

```bash
python3 webui/session.py --list
python3 webui/session.py --print-argv explore_resume        # prints the exact command
python3 webui/session.py --print-argv explore_fresh --set headless=true
```

`status: planned` is for scenarios whose environment doesn't exist yet — the
card lists them greyed out with a tooltip rather than pretending they run. One
known gap for the planned test suite: a person-avoidance scene needs a new
`--scene` value with an animated actor in `scripts/spawn_warehouse.py`.

Multi-robot coordination (`explore_fleet`, `robots: 2`) is `status: ready` —
`run_demo.sh` derives `--robot-ids` from `--robots` against
`configs/robots.yaml`'s order and threads it through to the bringup's
`robot_ids:=`, which brings up a full stack (RTAB-Map + Nav2 + task_manager +
explorer) per robot. See "Multi-robot" below.

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

> **Every `/snapshot` request to web_video_server MUST carry
> `qos_profile=sensor_data`.** web_video_server opens a fresh subscription per
> snapshot request, and its default is RELIABLE; that per-request churn of
> RELIABLE subscriptions kills Isaac's rgb/depth image writers outright at
> around sim-t=100–140 s (the sim keeps stepping, `camera_info` — a different
> publisher — keeps going, and Isaac logs nothing). Bisected live 2026-08-01:
> Isaac alone is stable indefinitely; the full stack minus web_video_server is
> stable; a steady RELIABLE subscriber (`depth_to_cloud`) is harmless; only
> the churning snapshot subscriptions correlate with the death. Both consumers
> (`webui/app.js`, `nodes/web_video_watchdog.sh`) already pass the parameter —
> keep it on any new one, including ad-hoc `curl` checks.

**Multi-robot:** `robot_ids:=robot_0,robot_1` brings up a full independent
RTAB-Map + Nav2 + `task_manager` + explorer stack per robot (the
dashboard/rosbridge stack stays singular). `scripts/run_demo.sh --robots N`
derives this list from `configs/robots.yaml`'s order automatically; pass
`--robot-ids` directly to override it. `localization`/`database_path`
(map-lifecycle overrides) still apply only to the single robot named by
`robot_id` (default `robot_0`) — every other robot in the list gets its own
auto-computed map path, which is already correct and collision-free without
an override. `explore`, unlike those, now applies to every robot in the list.
TF needs no changes for this — frames are already `<robot>/`-prefixed on the
shared global `/tf` (`scripts/_ros2_graphs.py`). `configs/nav2_params.yaml`
hardcodes a few frame ids/one topic to `robot_0`; `sortbots_nav2.launch.py`
rewrites them per-robot at launch time (see that file's docstring for why
it's plain string substitution, not `nav2_common.RewrittenYaml`).

Each robot's RTAB-Map owns its own SLAM grid, in its own `<robot_id>/map`
frame (see `sortbots_rtabmap_robot.launch.py`'s `map_frame_id`) — this is
still NOT collaborative SLAM (no shared pose graph, no inter-robot loop
closure). What ties the fleet together is `nodes/map_merge.py`: the bringup
publishes a static `map -> <robot_id>/map` transform per robot from its known
spawn pose (`configs/robots.yaml`, keyed by the `scene` launch arg — must
match `spawn_warehouse.py --scene`), and `map_merge.py` uses those anchors to
resample every robot's grid into one world-anchored `/map` — occupied beats
free beats unknown where two robots' grids disagree about a cell. Nav2's
`global_frame: map` and every explorer's default `--map-topic /map` both
point at this fused grid, so a robot plans and picks frontiers using space
ANY robot has mapped. See "Autonomous exploration" below for the
frontier-claim-sharing layer on top of that.

### What the dashboard shows

Open the printed URL (`http://localhost:8081/` locally, or the tailnet URL).
The page is sized to fit one viewport — nothing scrolls except the task queue.

**The stage** (the big box on the left) shows one view at a time, with an
optional picture-in-picture inset:

- **Chase cam** (default) large, no inset. The head camera still renders in
  Isaac (RTAB-Map needs it) but the dashboard no longer polls it as a PiP —
  that stream was browser-side load for a view nobody was watching.
- **Map mode** — the `camera` / `map` buttons in the header swap the map into
  the stage in place of the cameras; the chase cam stays as the inset so you
  don't lose sight of the robot while picking a goal. Switching back restores
  chase cam.
- **Drive pad** (bottom-left) and **head-aim pad** (bottom-right) float over the
  stage as translucent overlays — they fade up on hover, or on tap on a phone.
  The aim pad is hidden in map mode, where head pan/tilt means nothing.

A feed that's off the stage stops being fetched, so map mode isn't paying for
the chase camera's full-size decode, and the head camera isn't polled at all
while it's hidden.

Fleet runs (`explore_fleet`) spawn a chase cam on **robot_0 only** by default
(`chase_cam_robots: 1`) — the dashboard shows that robot's feed, so robot_1's
3rd-person cam would be a wasted render product. Override with
`chase_cam_robots=2` (or `=0` / `chase_cam=false`) via `sim_ctl.sh` if needed.

**The right column** is always visible: the **3D reconstruction** viewer on top,
**dispatch + task queue** below.

- **3D reconstruction** — RTAB-Map's assembled 3D map, orbit with the mouse
  (drag to orbit, scroll to zoom, right-drag to pan), "reset view" reframes it.
  Two selects:
  - **voxels / points** — voxel cubes are lit, so vertical structure actually
    reads as structure; points are cheaper and are also the automatic fallback
    above 250k points.
  - **photo / height** — the cloud's own colour, or a blue→red ramp by height,
    which is far more legible when everything is warehouse-grey.

  The panel subscribes to `/<robot>/recon_cloud`, not `cloud_map` directly —
  `nodes/recon_cloud_relay.py` sits in between and enforces a hard 200k-point
  budget. `cloud_map` grows without bound as the map does, and the vendored
  roslib can't reassemble fragments once a message passes rosbridge's
  `max_message_size`, so a budget is the only thing that keeps the panel
  working on a long run. The info line reports the z range, which is the
  quickest way to confirm the reconstruction is genuinely 3D.

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
- **Robot switcher** (header, next to the title) — populated from
  `configs/robots.yaml` via `GET /api/robots`. Picking a different robot
  reloads the page with `?robot=<id>` — every subscription on the page is
  robot-specific, so this is a reload, not a live hot-swap.

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

## Autonomous exploration

```bash
scripts/run_demo.sh --explore              # hands-off: starts exploring immediately
scripts/run_demo.sh --explore --resume     # EXTEND an existing map instead of starting fresh
```

(`--resume`, not `--localize`: the latter is read-only and never grows the
map, so exploring against it just re-walks what was already known. See
"Resuming exploration into an existing map" below.)

`nodes/explorer.py` finds frontiers (free cells bordering unknown space) in
`/robot_0/map`, drives `navigate_to_pose` toward the best-scored one, and
repeats until none remain reachable — then reports "exploration done" and
stops. No waypoints, no human input.

**Motion is deliberately diff-drive, not holonomic.** The sim base can
physically strafe, but Nav2 now runs MPPI with `motion_model: DiffDrive`
(`configs/nav2_params.yaml`) — the earlier holonomic DWB setup made the
robot glide sideways, which looked unnatural and pointed the only obstacle
sensor (the forward depth camera) away from the direction of travel. Brief
reverse (−0.15 m/s) is retained so BackUp recovery and corner escapes work.
Teleop strafing (`q`/`e` in `wasd_teleop.py`) is unaffected — the base is
still holonomic, Nav2 just never commands `linear.y`.

**Every Nav2 recovery depends on one easily-missed parameter.**
`behavior_server` reads *three* frames, not two: `global_frame`,
`robot_base_frame`, and `local_frame` — the odom-ish frame Spin, BackUp,
DriveOnHeading and AssistedTeleop all integrate against. It defaults to a
bare `odom`, which does not exist in a namespaced TF tree, so leaving it
unset makes every recovery abort instantly with `No Transform available …
"odom" does not exist` and turns the behavior tree's whole recovery branch
into a no-op. That is not a subtle degradation: a wedged robot can then
never physically free itself, and one exploration run failed 13 consecutive
goals without moving. `configs/nav2_params.yaml` now sets
`local_frame: robot_0/odom` (rewritten per robot by
`sortbots_nav2.launch.py`). Same class of bug as the static layer's
`map_topic` — an unqualified default that silently no-ops under namespacing.
The explorer's startup spin (below) exists partly to surface a regression
here on every single run.

**Speed: the robot stops driving to places it has already seen.** The depth
camera reveals a frontier from 4–5 m out (RTAB-Map `Grid/RangeMax 5.0`,
costmap `raytrace_max_range 5.0`), but a standoff goal sits only 1.5 m from
it — so the last several metres of every goal, plus a rotate-in-place to
satisfy `yaw_goal_tolerance`, were spent arriving to look at mapped space.
Now, once no frontier cell remains within `frontier_consumed_radius_m` of
the point a goal was chosen to reveal, the goal counts as **satisfied**:
cancelled and replaced immediately, with no blacklist entry and no failure
count (log line: `frontier at (…) already revealed en route to (…) —
retargeting (not a failure)`; counter `goals_consumed` in
`explore_status`). Two guards stop it thrashing — `goal_min_age_s`, and an
exact requirement that a new `/map` has arrived since the goal was sent,
since on the grid that produced the candidate the check could only ever say
"still there". If `goals_consumed / goals_sent` climbs above ~0.5 while the
robot barely travels, `frontier_consumed_radius_m` is too large.

### Steering exploration by hand

The map's **steer** mode (the `nav goal | steer` buttons above the map) lets
you direct exploration without stopping it. A click publishes
`{"x": .., "y": ..}` on `<robot>/explore_hint`; shift-click adds `"append":
true` to QUEUE an area instead of replacing the current one. Publish `{}` to
clear the queue.

While a hint is live the explorer heads for whichever frontier is **nearest
your click**, ignoring `max_goal_distance_m`, and keeps working that region
until it is mapped. It is still not a goal: the explorer picks real frontiers,
so an unreachable click cannot strand it, and clearance/validity masks are
hard filters applied before any of this.

Three ways a region ends, all bounded:

- the area is mapped — no unknown cells remain within `steer_radius_m` of the
  click, so the queue advances (`explore_hint: ... is mapped`);
- `steer_ttl_s` expires (the TTL runs from when a region becomes ACTIVE, not
  when you clicked, so queued entries cannot expire while waiting);
- the frontiers there prove unreachable and the sticky blacklist retires them.

A click **preempts the goal in flight immediately** (~10 ms measured).
Shift-click deliberately does not — queueing means "after this one".

Four things this got wrong before, all of which are instructive:

1. A pure score multiplier (`score *= 1 + steer_weight * exp(-d/steer_decay_m)`)
   loses to the distance term for any far click: at `alpha 1.0` a frontier 2 m
   away already beats an equal one 15 m away by 7.5x. It honoured a click for
   one goal and then wandered off. `steer_weight`/`steer_decay_m` still exist
   and still apply, but the region lock is what makes intent decisive.
2. Restricting candidates to those within `steer_radius_m` of the click looks
   right and is not: a frontier is a free cell ADJACENT TO unknown, so it sits
   at the EDGE of unexplored space, never inside it. Clicking into an unmapped
   area — the obvious gesture — found no frontier nearby and released
   instantly. Hence "nearest to the hint", and a release test on UNKNOWN
   cells rather than on frontiers.
3. Escape mode bypasses the scored candidate list entirely, so it discarded
   hints for as long as it was engaged — which, in a run where goals keep
   failing, is most of the time. It now uses the same nearest-to-hint rule.
4. Acting on a hint only at the next replan meant it queued behind the active
   goal, which could hold the robot for up to `goal_timeout_s` (75 s observed).

The queue draws on the map as dashed cyan rings — bright for the region being
worked, dimmer and numbered for those waiting — mirrored from `explore_status`
so they disappear as regions are finished or expire.

**`nav goal` mode stops exploration.** `navigate_to_pose` is a single-goal
server, so a manual goal and the explorer otherwise preempt each other every
second or so — and each preemption returns to the explorer as ABORTED, which
it scores as its own failure and blacklists, silently poisoning good frontiers
and driving spurious escapes. Click-to-nav therefore publishes
`explore_cmd=stop` first, exactly as `task_manager.py` does before dispatching.
The map mode is remembered across reloads, because the two modes look
identical to click and do opposite things.

### Seeing why the explorer refuses an area

The **blacklist** toggle next to `costmap` draws every blacklist entry at its
real suppression radius: dashed orange for entries that will expire, solid red
for permanent ones, with the strike count in the middle. Worth having because
the escalating blacklist makes "2 strikes, expires in 6 minutes" and "3
strikes, gone for good" otherwise indistinguishable. Off by default — it is a
diagnostic, not demo chrome.

**Corner/dead-end handling** (all tunable in `configs/explorer.yaml`):

- *Startup spin* — one 2π rotation before the first frontier goal. At t=0 a
  forward-only camera has seen a cone, so the first choice is close to a
  guess, and frequently the *only* candidate. ~8 s of sim time buys a full
  `Grid/RangeMax` disc. It doubles as a per-run check that recoveries work
  at all (see `local_frame` above) — watch for `startup spin complete —
  recoveries are functional`.
- *Standoff goals* — the goal is pulled back from the frontier cell into
  known free floor with clearance from walls **and** unknown space, posed
  facing the frontier; the camera reveals it from the standoff instead of
  the robot driving nose-first into an unmapped corner.
- *Stuck watchdog* — less than 0.15 m of net motion over 15 s with a goal
  active ⇒ blacklist + retarget immediately (log line: `stuck: moved
  <0.15m …`) instead of waiting out `goal_timeout_s` or Nav2's
  multi-minute recovery churn.
- *Openness scoring* — frontier score is weighted by the fraction of free
  floor around the goal, so open-floor frontiers beat tight corners at
  similar size/distance.
- *Backtracking* — when no frontier is within `max_goal_distance_m`
  (typical after fully mapping a dead-end pocket), the explorer targets the
  nearest remaining frontier anywhere on the map rather than declaring
  exploration done. No DFS-style path memory needed: frontier selection is
  global and Nav2 plans the route back out through known space.
- *Escape mode* — after `escape_after_failures` (default 2) consecutive
  failed goals (stuck/timeout/abort), the explorer stops nibbling at the
  pocket it's wedged in — blacklisting removes failed spots one at a time,
  so nearest-first scoring would otherwise keep picking the next frontier
  in the same pocket — and jumps to the **farthest** valid frontier on the
  explored boundary (log line: `escape: N consecutive failed goals …`),
  ignoring the distance cap. Normal nearest-biased scoring resumes on the
  next success. Escape also *pocket-blacklists* a
  `escape_pocket_radius_m` disc around the robot, so the pocket dies in one
  go rather than one 0.5 m spot per failure — applied only when the escape
  target lies outside that disc, so it never blacklists its own destination.
- *Hard escape* — at twice `escape_after_failures` (failed repeatedly **and**
  failed to escape) the explorer spins in place before retargeting. That
  combination almost always means a physical wedge with a stale local
  costmap, and rotating is the highest-information action a forward-only
  camera has. Rate-limited by `hard_escape_cooldown_s`, which is **not
  optional**: the trigger is a level (the failure count stays high until a
  goal succeeds), not an edge, so without a minimum gap the spin re-fires the
  instant the previous one ends. That produced a live 8-second
  spin/complete/spin loop that replaced exploration entirely.
- *What counts as success* — early-abandoned goals reset the consecutive-
  failure count **if the robot actually travelled** (`>
  stuck_min_displacement_m`) to earn the reveal. This matters more than it
  sounds: abandonment ends ~85% of goals in practice, so `STATUS_SUCCEEDED`
  almost never fires, and a failure counter that only reset on it climbed
  monotonically into permanent escape mode. Requiring real displacement is
  what separates "revealed the frontier by driving there" from "sat still
  while it evaporated", so a genuinely wedged robot still escalates.
- *Sticky blacklist* — some frontiers are genuinely unreachable (behind a
  rack, behind sim geometry, in a pocket the planner can't route into). With
  a flat TTL they returned every `blacklist_ttl_s` forever, which both burned
  goal cycles and made "exploration done" **structurally unreachable** —
  there was always one more candidate, so the empty-cycle counter could never
  run up. Re-blacklisting near an existing entry now bumps its *strike* count
  instead of adding a neighbour; each strike multiplies its TTL
  (`blacklist_ttl_growth`) and widens its radius, and at
  `blacklist_permanent_strikes` it stops expiring. An unreachable frontier
  costs three failure cycles and is then gone for good.
- *Run-time budget* — `max_run_time_s` (default 2400 s wall) ends the run
  regardless. Every other termination path depends on the frontier set going
  empty, which a stalled perception stack can prevent indefinitely; this
  guarantees the run finishes and the final map gets saved. A lost
  `map → base_link` transform is logged as an error after 30 consecutive
  ticks and force-finishes the run after 300, since that path otherwise
  hangs silently in `exploring` forever.

When exploration ends, the explorer logs a run summary — elapsed time, grid
size, known/free/occupied m², coverage against the reference map, goal
counters (`sent / reached / failed / consumed-early / rejected`) and the
blacklist size, including how many entries went permanent.

### Reacting to moved/moving obstacles

D* Lite was considered for this and deliberately **not** implemented: the
stock Nav2 behavior tree already replanned the global path from scratch at
1 Hz unconditionally, and NavFn A* on this grid costs single-digit
milliseconds — an incremental planner has nothing to save. The actual
bottlenecks were perception latency and the lack of a blocked-path trigger,
which is what these three changes fix (worst-case reaction to a
newly-blocked path drops from ~2 s to ~0.5 s):

- **Sensing ranges** (`configs/nav2_params.yaml`): both costmaps mark
  obstacles out to 4 m and raytrace-clear out to 5 m (Nav2's unset defaults
  were 2.5/3.0 m). Still forward-frustum-only — a stale mark behind the
  robot persists until the camera sweeps over it again; that's a sensor
  limitation, not a config choice.
- **Global costmap rate**: 1 Hz → 4 Hz updates (2 Hz publish), so a new
  obstacle reaches the global planner in ≤250 ms.
- **Reactive behavior tree** (`configs/bt/navigate_to_pose_reactive.xml`,
  injected by `sortbots_nav2.launch.py` as a bt_navigator parameter
  override): checks the current path against the costmap at 4 Hz via
  planner_server's `/is_path_valid` and replans **immediately** when a
  lethal cell lands on it, refreshes every 3 s otherwise, replans on goal
  change, and keeps the stock recovery ladder. Unknown cells don't
  invalidate a path, so frontier goals through unmapped space don't cause
  replan thrash during exploration.

Quick test without touching the sim setup: while the robot is en route on a
long goal, drop a cube (~0.5 m, center ~0.25 m high so it sits in the
0.05–1.5 m marking band) onto the planned path 2–4 m ahead via the Isaac
GUI. Expect the costmap to mark it within ~0.25 s and a new `/robot_0/plan`
around it within ~0.5 s (log: "Passing new path to controller" with no goal
change) — MPPI swerves without entering recovery. Delete the cube while
it's still in view and the path straightens within ≤3 s. Teleopping robot_1
across robot_0's path works too (and crossing *behind* robot_0 demonstrates
the no-rear-sensing ghost caveat).

**Requires the occupancy-grid tuning documented above** (`GRID_ARGS` /
`Grid/RayTracing`) — frontier detection needs real free space in the map to
find a boundary to chase at all.

From the dashboard's **Dispatch & task queue** panel: **Start**/**Stop**
control exploration directly (`<robot>/explore_cmd`), the status line shows
state / current goal / % of the grid mapped / blacklisted-goal count
(`<robot>/explore_status`), and frontier candidates appear as colored dots on
the map view (brighter = the one currently being pursued). **Save map**
triggers RTAB-Map's own `backup` service for a labeled snapshot — the
database is already being written continuously as it maps (see above), this
just gives you a specific point-in-time copy.

`task_manager.py` and `explorer.py` both drive the single `navigate_to_pose`
action server, so they arbitrate: `task_manager` won't pop a dispatched task
off its queue while the explorer reports `state: "exploring"`, and dispatching
one anyway sends `explore_cmd: stop` first. Manual click-to-nav on the
dashboard still preempts everything unconditionally — that's a deliberate
override, same as always.

**Tuning** lives in `configs/explorer.yaml` (frontier size cutoff, goal
scoring, blacklist/claim radii and TTLs, replan cadence, done-after-N-empty
cycles). Both were widened from their original values after live testing —
this was the pre-flagged "SimpleProgressChecker will abort frontier goals
too aggressively" risk from the exploration-testbed plan, and it showed up
exactly as predicted:

- `configs/nav2_params.yaml`'s `goal_checker.xy_goal_tolerance` was the real
  culprit, not the progress checker. A frontier goal sits right at the edge
  of currently-known space by definition, which is a harder final-approach
  target than an interior waypoint — measured live, the robot routinely
  settled ~0.2-0.25 m short and then just sat there never satisfying the
  original 0.15 m tolerance. Widened to 0.25 m; goals started succeeding
  immediately (three in a row on the very next run). Still tight enough for
  `task_manager`'s dispatched pick/place docking, which has its own 0.5 m
  `dock_offset_m` standoff on top of this.
- `movement_time_allowance` (10.0 → 20.0) — the base needs a brief
  rotate-in-place before committing to a heading on a long traverse (even
  more so now that it's driven diff-drive), which the original tight window
  read as "no progress" and aborted early.
- `explorer.yaml`'s own `goal_timeout_s` (45 → 75) is a backstop for
  whatever the two Nav2-level settings above don't catch; with the tolerance
  fix it should now fire rarely.

**Multi-robot claim sharing:** every explorer also publishes/subscribes a
global (not robot-namespaced) `/explore/claims` topic — "I'm heading here,
don't also send your robot to this frontier." Each robot still builds and
owns its own RTAB-Map database independently (not a shared pose graph — see
"Multi-robot" above for how `nodes/map_merge.py` fuses the resulting grids
into one `/map` instead); claim-sharing is the coordination layer on top of
that shared grid. A claim is refreshed every replan tick its goal stays
healthy, released explicitly the moment that goal ends (success, failure, or
preemption — not left to expire), and published `TRANSIENT_LOCAL` so a robot
that starts exploring after its peers still sees their current claims
immediately. Frontier candidates near another robot's current TF pose are
also excluded, not just near its claimed goal — a claim marks where a peer
is headed, not the path it's driving to get there.

### The two action-server races this was built against

Both bit real test runs before being fixed — worth knowing if you extend
`explorer.py` or `task_manager.py`:

1. **Don't explicitly cancel a goal you're about to replace.** Nav2's
   `navigate_to_pose` is a single-goal action server: sending a new goal
   already preempts whatever was running. An explicit `cancel_goal_async()`
   on the OLD goal, fired right before sending the new one, is processed
   asynchronously and reliably arrives *after* the new goal has already
   preempted the old one — canceling the new goal instead. `explorer.py`
   only cancels on an explicit `stop` (`nodes/explorer.py`'s
   `_cancel_active_goal(cancel_on_server=...)`), never on its own
   goal-timeout-and-replace path.
2. **Goal-response/result callbacks must not read mutable instance state.**
   A preempted goal's result future still completes later (as
   ABORTED/CANCELED). If its callback reads `self._goal_target` instead of a
   value captured at send time, it reads whatever the *current* goal is by
   then — misattributing a stale failure to a goal that's still actively
   navigating, and blacklisting a perfectly good target. `explorer.py` fixes
   this with a generation counter: each goal's callbacks capture their own
   generation number and no-op if a newer goal has since been sent.

## Map lifecycle: build a map once, navigate on it later

By default every run builds a **fresh** map — RTAB-Map starts from an empty
database, so a mapping run always means what it says. The map persists after
shutdown, and `--localize` reopens it read-only instead of rebuilding:

```bash
scripts/run_demo.sh --teleop      # drive around; map builds from scratch
scripts/run_demo.sh stop          # map persists on disk
scripts/run_demo.sh --localize    # reopen it, localize, send nav goals
```

The database lives at `~/.ros/sortbots_<robot_id>.db` — **per robot**, so two
robots can't clobber each other's map. Override with `--map PATH` to keep
several warehouses around. `--localize` refuses up front if the file doesn't
exist, rather than coming up and silently rejecting every goal.

In localization mode RTAB-Map sets `Mem/IncrementalMemory=false`, so the map is
never modified — verified by checksum across a full run. `--delete_db_on_start`
is forced off whenever `localization:=true`; deleting the map you are about to
localize against is a footgun, not a preference.

Nav2 needs no changes between any of these modes: its static layer consumes
the same `map` topic regardless.

### Saving a map off a run

Two artifacts are worth keeping, and they have different lifetimes: the
**occupancy grid** (`.pgm` + `.yaml`) can only be captured while the stack is
up, since it comes off the live `/map` topic; the **RTAB-Map database**
(`.db`) can only be copied safely once the stack is *down*, because copying a
live sqlite file mid-write gives a torn database. `scripts/save_map.sh` does
both (needs ROS 2 sourced, unlike `run_demo.sh`):

```bash
scripts/save_map.sh --run nvidia_explore_20260802 --watch 3 &   # checkpoint every 3 min
# ... explore until "exploration done" ...
scripts/save_map.sh --run nvidia_explore_20260802 --label final
scripts/save_map.sh --stop-watch
scripts/run_demo.sh stop
scripts/save_map.sh --run nvidia_explore_20260802 --db          # only now
```

Everything lands in `data/runs/<name>/map/`. The `--watch` loop exists for
two reasons: `run_demo.sh stop` `pkill -9`s rtabmap only 2 s after SIGINT and
can take the last minutes of mapping with it, and the checkpoint series
doubles as a coverage-versus-time curve. It is deliberately *not* in
`run_demo.sh`'s `PIPELINE_PATTERNS`, so teardown doesn't kill it before the
final checkpoint lands; it ends itself once `/map` stops answering.
`--occ`/`--free` are passed explicitly rather than left to `map_saver`'s
defaults — unspecified thresholds are why this repo already has saved yamls
disagreeing about `free_thresh` (0.25 vs 0.196).

### How much of the warehouse is actually mapped

```bash
scripts/map_coverage.py data/runs/<name>/map/final.yaml \
    --reference data/runs/nvidia_explore_20260801_145700/map/checkpoint_resume002.yaml
scripts/map_coverage.py --live --watch 30      # against the running stack
```

Reports free / occupied / unknown / known in both cells and m². The file mode
is ROS-free on purpose, so it still works after teardown.

**`explored_pct` in the dashboard is not coverage.** It divides known cells
by the *current* grid's own extent, and that extent grows as RTAB-Map
explores — so it can fall while the robot is making progress. On the most
complete map this repo has built it reads **43.8%**. Use `coverage_pct` /
`free_area_m2` from `explore_status` instead, or this script. The denominator
is `reference_free_area_m2` in `configs/explorer.yaml` (293.7 m², the free
floor of that reference map). It is a ratio of scalar *areas*, deliberately
not a cell-wise overlay — two RTAB-Map sessions have no common frame — and
exceeding 100% just means this run mapped more than the reference did.

One trap worth knowing if you write your own parser: in `mode: trinary`,
classify by `map_saver`'s sentinel pixels (254 free / 205 unknown / 0
occupied), *not* by the yaml thresholds. Unknown's implied occupancy is
0.196078 and this repo's own saved yamls carry `free_thresh: 0.196` — 8e-5 of
margin. A yaml written with `free_thresh: 0.2` would silently reclassify
every unknown cell as free and report ~100% coverage on a half-explored map.

### Resuming exploration into an existing map

`--localize` is deliberately read-only (`Mem/IncrementalMemory=false`) — it
will never reveal new territory, so autonomous exploration against a
`--localize` session just re-walks whatever was already known. To actually
**continue exploring** — extend a map from a prior session instead of either
starting fresh or being stuck read-only — use `--resume`:

```bash
scripts/run_demo.sh --resume --explore --map data/runs/<name>/map/rtabmap.db
```

`--resume` is mapping mode (RTAB-Map keeps growing the pose graph, same as a
plain fresh run) but with `delete_db_on_start:=false`, so it starts from the
existing database instead of wiping it. `--localize` and `--resume` are
mutually exclusive (read-only vs. keep-mapping). The explorer's blacklist is
in-memory only and does not persist across restarts — a fresh `explorer.py`
process re-evaluates every current frontier from a clean slate, which is
usually desirable (a previously-blacklisted point that's since been driven
past may well be reachable now).

### Occupancy-grid tuning (why the map has usable free space)

`launch/sortbots_rtabmap_robot.launch.py`'s `GRID_ARGS` passes RTAB-Map
`--Grid/3D true --Grid/RayTracing true --Grid/RangeMax 5.0
--Grid/MaxObstacleHeight 1.5`. Ray tracing is the load-bearing one: without it
the only cells marked free are ones where a depth point happened to land on the
floor, so `/map` stays overwhelmingly unknown even in rooms the robot drove
through.

`Grid/3D` used to be `false` here, and this section used to claim that was
"what lets ray tracing work at all". That was wrong, and it cost the dashboard
its 3D panel — with 2D local grids, `cloud_map` is just the occupancy grid
re-emitted as points, a ~0.15 m-tall pancake, and every `octomap_*` topic
publishes empty because `OctoMap` skips any node whose local grid isn't 3D.
The misreading was of `Grid/RayTracing`'s doc string: *"if `Grid/3D=true`,
RTAB-Map should be **built with** OctoMap support, otherwise 3D ray tracing is
ignored"* is a **build precondition**, not a prohibition — and this build
satisfies it. In fact `Grid/3D` defaults to `true` on an OctoMap-enabled build
(`Parameters.h:860-864` picks the default under `#ifdef RTABMAP_OCTOMAP`), so
`false` was overriding the build's own default. The 2D `/map` is unaffected —
`Grid/3D`'s own doc string says "a 2D map can be still generated if checked",
and `OccupancyGrid::assemble()` transforms each cell with its real z and then
keeps only x/y.

What `Grid/3D=true` does cost is memory and time: empty cells go from O(area)
to O(volume) per node. `Grid/RangeMax` is what bounds that volume, so it is the
first lever to reach for if RTAB-Map starts eating RAM (then
`Grid/DepthDecimation`, and only last `Grid/CellSize`, which changes `/map`'s
resolution and so also Nav2's static layer and the explorer's
`min_frontier_cells`).

Measured on the primitive scene after ~80 s of driving: **52.9 m² free / 54.8%
of the grid**, versus a near-empty map before. That matters because frontier
exploration is literally "a free cell adjacent to an unknown cell" — with no
believable free space there is nothing real to explore toward.

Override with `rtabmap_args:=...`, but note it replaces the tuning wholesale.

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

The Scenarios tab has its own test, which needs **no fixture** — it talks only
to `serve.py`'s control API, so the real thing runs entirely offline:

```bash
node webui/tests/scenarios_test.mjs                  # add --screenshot DIR for PNGs
```

It runs `serve.py` twice (with and without `--control`, since "the console
isn't running" is a state the tab has to explain rather than fail in) and
asserts the view switch hides the live view and its header chrome, that a card
renders per scenario with its override controls, that only `ready` scenarios
are startable, that the camera pollers idle behind the hidden panel, and that
nothing scrolls horizontally at three viewports. It never POSTs
`/api/session/start`, so it cannot launch a sim.

### 4. Replay through the real stack (optional)

To eyeball it in a real browser, with real rosbridge and real MJPEG:

```bash
scripts/replay_dashboard_bag.sh data/bags/dashboard_<timestamp>/
# then open http://localhost:8081/
```

This is also how to double-check fixture fidelity: what the browser gets here
went through the live rosbridge serializer, which is the same `extract_values`
the fixture was built with.
