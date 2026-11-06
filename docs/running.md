# Running the sim

## Launching

### Console mode (day to day)

From a clean shell:

```bash
scripts/run_console.sh          # rosbridge :9090, web_video_server :8080, serve.py :8081. Leave it running.
```

Open <http://localhost:8081/>, click **Session**, pick a scenario and press
**Start**. The panel streams the launcher's log and its phase (`starting Isaac Sim →
loading the warehouse → sim publishing → starting RTAB-Map + Nav2 → running`).
**Save & finish** saves the map into `maps/` and stops the run. **Stop** stops
without saving. Either way the console stays up. `scripts/run_console.sh stop` takes
down the console and any running sim.

Each run logs to `data/sessions/<timestamp>_<scenario>/`. A restarted console
re-adopts a sim that is still running.

### Scripted (`sim_ctl.sh`, for automation and agents)

Exit codes are the interface: `0` ok, `1` failure, `3` no console, `4` nothing
running, `124` timeout.

```bash
scripts/sim_ctl.sh console start                  # detached
scripts/sim_ctl.sh list                           # scenarios
scripts/sim_ctl.sh dry-run explore_fleet          # print the command, launch nothing
scripts/sim_ctl.sh start explore_fleet headless=true
scripts/sim_ctl.sh start custom map=warehouse mode=extend robots=2   # the launch form, as k=v
scripts/sim_ctl.sh wait running --timeout 420     # first run streams assets
scripts/sim_ctl.sh status
scripts/sim_ctl.sh log --lines 100
scripts/sim_ctl.sh stop [--save-map NAME [--splat]]
```

### CLI only (`run_demo.sh`)

```bash
scripts/run_demo.sh                 # 2-robot fleet + dashboard
scripts/run_demo.sh --explore       # ...exploring autonomously
scripts/run_demo.sh --robots 1      # single robot
scripts/run_demo.sh --teleop        # add a WASD terminal window
scripts/run_demo.sh stop
```

`run_demo.sh` returns as soon as everything has launched. Check liveness with
`pgrep -f spawn_warehouse.py`, not its exit code.

### Scenarios (`configs/scenarios/`)

| Scenario | What it does |
|---|---|
| `explore_fleet` | **default**: 2 robots explore from empty maps, fused into one `/map` |
| `explore_fresh` / `explore_resume` | 1 robot, fresh map / extend the working map |
| `library_localize` / `library_resume` | a saved map from `maps/`, read-only / keep mapping |
| `fleet_demo` | premapped warehouse, 2 robots, for the task demo |
| `fleet_scale` | 4+ robots waiting for goals (`robots=` overridable) |
| `moving_obstacles`, `_mapped`, `_fleet` | worlds with scripted walkers, for watching live and for tests |
| `pick_bench` | fresh map, no Nav2 goals: the base for pick and place tests ([Pick and place](manipulation.md#pick-and-place-sim)) |
| `real_map_fresh`, `real_localize`, `real_map_offload` | the Jetson (`platform: real`) |

A scenario is one YAML file whose filename matches `name`. `run:` accepts only
allowlisted keys (`RUN_FLAGS` in `webui/session.py`). Unknown keys make the scenario
`invalid` rather than being passed through.

```yaml
name: explore_fresh
title: Frontier exploration — fresh map
description: Shown in the launch form.
status: ready                 # ready | planned
run:
  scene: nvidia               # nvidia | primitive
  robots: 1
  headless: true
  chase_cam: true
  explore: true
  resume: false               # resume / localize are mutually exclusive
  localize: false
  map: null                   # null → ~/.ros/sortbots_<robot_id>.db
overrides: [headless, robots] # keys the launch form may change
capture: {bag: false}
```

Check scenarios offline with `python3 webui/session.py --list` or `--print-argv NAME
[--set k=v]`. **Save as preset…** in the launch form writes a new one.

### Performance knobs

- **Render products dominate cost** (about 85%). Chase cams are off for fleet runs by
  default. `half_res_cams` renders head cameras at 424×240.
- On a GPU-bound laptop, `render_every=2` + `lite_render=true` took `explore_fleet`
  from 0.38× to 0.81×. A CPU-bound desktop only speeds up with fewer render products.
- **Memory:** Isaac plus the console take about 6.8 GB. Each robot adds about
  1.5 GB, plus RTAB-Map growth of about 3.5 MB per sim-second while mapping.
  Past 4 robots the limit is CPU at startup, not RAM. `run_demo.sh` runs the
  pipeline in a systemd memory fence, and `scripts/mem_guard.py` stops the run
  cleanly before the OOM killer acts (`SORTBOTS_NO_MEM_GUARD=1` disables it).
  `scripts/mem_probe.py record|summarize` breaks usage down per robot.

### Always-on console and remote access

```bash
scripts/console_service.sh install    # systemd user service serving this checkout
scripts/console_service.sh deploy     # serve a worktree of main (~/SortBots-prod), auto-updated every 5 min
python3 scripts/webui_url.py          # print the tailnet URL
```

Every port binds `0.0.0.0`, so any device on the Tailscale tailnet can reach the
operator view at `http://<host>.ts.net:8081/`. Use plain http there: the page
derives its rosbridge and video hosts from its own origin. For a public URL, Tailscale
Funnel can expose 8081→443, 9090→8443 and 8080→10000. **The control API is not
authenticated, and rosbridge can publish to any topic.** Tailnet ACLs are the only
perimeter, so treat a Funnel URL as a secret.

### The ROS side without the launcher

With Isaac already running (`scripts/spawn_warehouse.py`), in a ROS-sourced shell:

```bash
ros2 launch ./launch/sortbots_bringup.launch.py robot_ids:=robot_0,robot_1 \
    [nav2:=false] [webui:=false] [task_manager:=false] [rviz:=true]
```

## The operator view

`webui/operator.html` is the **only** GUI. New UI belongs inside it as a view,
panel or popover. Never add a second page or port.

- **Header:** pick the stage (**Map** / **3D cloud** / **Sim**), the **Comms**
  overlay, **Session** (the launch form), **Stop all** (cancels every goal and
  zeroes `cmd_vel`), and the alarm chip.
- **Robot cards (left):** camera/depth tile, current task, pose, speed and the
  distance left on the plan. The card used most recently has control: map clicks
  and waypoint chips go to that robot.
- **Stage (centre):**
  - 2D: the fused `/map` with robot poses, Nav2 plans, waypoints and select /
    waypoint / send-robot tools (drag to set heading).
  - 3D: `/fleet/recon_cloud`, throttled to one cloud per 8 s.
  - **Sim:** Isaac's own floor camera. It renders only while a page sends
    heartbeats; drag to orbit, **Follow** to ride along.
- **Tasks (right):** per-robot FIFO queues from `task_manager`. **Moves** queues
  ordered `goto`s. **Pick & drop** queues pickup→dropoff. **Any robot** sends the
  task to the fleet auction. **Run demo** runs `configs/demo_tasks.yaml` in phases.
- **Exploring:** each robot card has an **Explore** / **Stop exploring** button.
  **Save & finish** in Session saves the run into `maps/` and then stops it.
- **Comms overlay:** commsim's radio model running live on the real poses and map:
  predicted links, mesh routes and message hops. What-ifs (pull a link, switch off
  a robot, range slider) change only the model, never the real fleet. Drop the
  gateway pin with ▲.

Test it offline. There is no sim, no ROS and no npm install; it uses a fake
ROSLIB in headless Chrome:

```bash
node webui/tests/operator_test.mjs [--screenshot DIR]
scripts/replay_dashboard_bag.sh <bag-dir>     # eyeball a recorded bag with real rosbridge
```

Never point a browser at a live sim to test the UI. Record a bag and replay it.

## Maps

| Mode | Flags | Pose graph |
|---|---|---|
| fresh | *(default)* | wiped and rebuilt |
| resume | `--resume` | reopened and extended |
| localize | `--localize` | reopened read-only (`Mem/IncrementalMemory=false`) |
| library | `--map maps/<name>/map.db` + `--resume`/`--localize` | **copied** to `~/.ros/sortbots_<id>.db`, and the copy is used |

The working database is `~/.ros/sortbots_<robot_id>.db`, one per robot. RTAB-Map
opens its sqlite file read-write even under `--localize`, which is why library maps
are always copied. Otherwise every demo would dirty a tracked git-lfs object.

**The library** (`maps/<name>/`: `map.json` manifest, `grid.pgm`/`grid.yaml`,
`map.db`, plus `map_<rid>.db` per extra robot):

```bash
scripts/maps.sh list | show NAME | verify NAME | rm NAME
scripts/sim_ctl.sh stop --save-map NAME            # save + stop in one step
scripts/sim_ctl.sh start library_localize map=$PWD/maps/NAME/map.db
```

Saving has two halves. The grid has to come off the live `/map` while the stack is
**up**. A safe copy of the database needs the stack **down**, or RTAB-Map's own
backup service. So `maps.sh save` is idempotent: a mid-run save can land as
`db_state: pending`, and a second save after teardown completes it (exit 5 means
pending). Names match `^[a-z0-9][a-z0-9_-]{0,63}$`. Entries are 100+ MB, so prefer
a new name over re-saving an old one.

**Several robots on one saved map:** every robot gets a copy and an initial pose
relative to robot_0. Each switches from localizing to mapping only after a real
visual match. Only robot_0's database is saved back.

Other tools: `scripts/save_map.sh` (per-run checkpoints under `data/runs/`) and
`scripts/map_coverage.py [--live]` (how much of the warehouse is mapped).
