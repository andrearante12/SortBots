---
name: sim-run
description: Start, monitor, or stop a SortBots simulation run, or set up a test environment for one. Use whenever the task involves running the sim, launching a scenario (exploration, mapping, navigation), bringing up the dashboard/console, checking whether a sim is already running, tearing one down, or adding a new scenario to the test suite. Also use before debugging anything that needs live ROS 2 topics, a map, Nav2, or camera feeds, since all of those require a running sim.
---

# Running a SortBots sim

Everything here goes through `scripts/sim_ctl.sh`. **Do not read the launch
files, `run_demo.sh`, or `spawn_warehouse.py` to work out how to start a run** —
that is the expensive path this skill exists to replace. Read them only when
changing how launching itself works.

## Non-negotiables

- **Run from a clean shell.** No `source /opt/ros/...`, no Isaac venv, no conda
  env. `run_demo.sh` and `activate_isaac.sh` both hard-refuse when
  `AMENT_PREFIX_PATH` is set, because Isaac's venv and system ROS 2 must never
  share a shell. If a command must have ROS sourced, source it in its own
  `bash -c` subshell, never in the shell you launch the sim from.
- **A sim run is minutes, not seconds.** Isaac takes ~30–90 s to come up warm,
  and **several minutes on the first ever run** (it streams the NVIDIA warehouse
  from the asset server). Always pass a generous `--timeout`; never conclude
  "it failed" from a short wait.
- **Only one sim at a time.** Starting a second run while one is live is
  rejected. Check `status` first.
- **Never leave a run going** when you are done with it. Isaac holds the GPU.

## The whole workflow

```bash
scripts/sim_ctl.sh status                 # ALWAYS start here
scripts/sim_ctl.sh console start          # if status said the console isn't up
scripts/sim_ctl.sh list                   # what scenarios exist
scripts/sim_ctl.sh start explore_fresh    # launch one (returns immediately)
scripts/sim_ctl.sh wait running --timeout 420
# ... do the actual work ...
scripts/sim_ctl.sh stop                   # leaves the console up for the next run
```

Branch on **exit codes**, not on the text:

| code | meaning |
|---|---|
| 0 | success / requested state reached |
| 1 | command failed (bad scenario, rejected start, run went to `failed`) |
| 3 | the console isn't running → `scripts/sim_ctl.sh console start` |
| 4 | no session exists (from `status` and `wait`) — this is how you ask "is anything running?" |
| 124 | timed out waiting |

`list` and `dry-run` never touch the console and work with nothing running.
`stop` and `log` are idempotent.

## Architecture, in three sentences

The **console** (`scripts/run_console.sh`, wrapped by `sim_ctl.sh console start`)
is the long-lived half: rosbridge 9090, web_video_server 8080, and
`webui/serve.py --control` on 8081. The **session** is one sim run —
`webui/session.py` shells out to `scripts/run_demo.sh --keep-console`, which
starts Isaac Sim plus the ROS 2 bringup (RTAB-Map, Nav2, task_manager) and
returns while they keep running under `setsid`. The console outlives sessions on
purpose, so the dashboard can start and stop the sim without killing the page
it's served from.

Consequence worth internalising: **`run_demo.sh` exiting 0 is the success path,
not the end of the run.** Liveness is a property of the pipeline
(`pgrep -f spawn_warehouse.py`), never of that pid.

## Scenarios

Presets in `configs/scenarios/*.yaml`, one file each. Today:

- `explore_fresh` — wipe the map, autonomous frontier exploration from empty
- `explore_resume` — extend the map a previous run built (**needs an existing
  `~/.ros/sortbots_robot_0.db`**, else the run fails immediately and says so)

Per-run overrides are `key=value` and only for keys the scenario lists under
`overrides:`:

```bash
scripts/sim_ctl.sh start explore_fresh headless=true robots=1
scripts/sim_ctl.sh dry-run explore_fresh headless=true   # prints the command, launches nothing
```

Prefer `headless=true` for unattended work — it drops the Isaac window; the
dashboard, map, and camera topics all still work.

### Adding a scenario

Copy `configs/scenarios/explore_fresh.yaml`, change `name` to match the new
filename stem, and edit `run:`. Only keys in `RUN_FLAGS` (`webui/session.py`)
are accepted — an unknown key makes the scenario load as `invalid` with the
reason rather than being forwarded to a shell. Verify with:

```bash
python3 webui/session.py --list
scripts/sim_ctl.sh dry-run <name>
```

Use `status: planned` for a scenario whose environment doesn't exist yet: it
lists in the dashboard, greyed out, instead of failing at launch. Two known
gaps — a person-avoidance scene needs a new `--scene` with an animated actor in
`scripts/spawn_warehouse.py`, and multi-robot coordination needs `run_demo.sh`
to thread `robot_ids` through to the bringup (it only passes `robot_id` today).

## When something goes wrong

`scripts/sim_ctl.sh wait` already dumps the last 30 log lines on failure. More:

```bash
scripts/sim_ctl.sh log --lines 100        # this session's run_demo.sh output
tail -100 /tmp/sortbots_demo_sim.log      # Isaac's own stdout — the real errors live here
tail -100 /tmp/sortbots_demo_bringup.log  # RTAB-Map / Nav2 / task_manager
tail -50  /tmp/sortbots_console.log       # rosbridge + web_video_server
```

Full session record: `data/sessions/<timestamp>_<scenario>/`.

Known failure modes, in rough order of likelihood:

- **`console start` times out** — something already holds 8081 or 9090. Check
  with `ss -ltn | grep -E '8081|9090'`; clear it with
  `scripts/sim_ctl.sh console stop`.
- **Stuck in `sim_loading` for minutes** — normal on a first run (asset
  streaming). Confirm progress in `/tmp/sortbots_demo_sim.log` before giving up.
- **`explore_resume` fails instantly** — no map database. Run `explore_fresh`
  first.
- **State flips to `exited`** — the pipeline died outside the dashboard (Isaac
  crash, OOM, someone ran `run_demo.sh stop`). Read the Isaac log.
- **`ModuleNotFoundError: rclpy._rclpy_pybind11`** — conda's `python3` is ahead
  of `/usr/bin` on PATH. Prepend the system path inside the offending subshell;
  see `launch/sortbots_webui.launch.py`'s docstring.
- **Camera panes dead but SLAM fine** — web_video_server livelocked; its
  watchdog restarts it. Any `/snapshot` request **must** carry
  `qos_profile=sensor_data`, including ad-hoc `curl` checks — a RELIABLE
  subscriber can kill Isaac's image writers outright.

## Testing without a sim

Prefer this. It is seconds instead of minutes, and needs no GPU or display.

```bash
node webui/tests/scenarios_test.mjs    # scenarios tab; needs no fixture
node webui/tests/dashboard_test.mjs    # live dashboard; needs webui/testdata/
python3 webui/session.py --list        # scenario validation, offline
```

Never point a browser at a live sim to test the dashboard — record a bag, build
a fixture, test offline (`docs/running.md`, "Testing the dashboard without the
sim").

## Reference

- `docs/running.md` — the full runbook this summarises
- `webui/session.py` — scenario schema, `RUN_FLAGS`, phase table
- `scripts/run_demo.sh` — what actually launches; its progress lines are parsed
  by `PHASE_PATTERNS` in `session.py`, so reword them only in the same commit
