# The Robot view: the real robot on a gamepad

The operator view's **Robot** view (next to Map / 3D cloud / Sim) runs the robot plugged into
this machine (the laptop today): the head D435 live, the omni base on a gamepad, and the arm's
one-button pick and place. It needs no ROS.

```bash
~/miniconda3/envs/lerobot/bin/python webui/serve.py --drive   # then open http://localhost:8081/
```

Open **Robot**, press **Start**, press any button on the pad (a browser lists a pad only after
one of its buttons is pressed).

On the **Jetson**, run the same serve.py on the Jetson's host and reach it through an SSH tunnel
from the laptop, with the ROS console stopped: `docs/real-robot.md`, "The arm, pick and place,
and the Robot view on the Jetson". A new machine needs the calibration first:
`scripts/calibration.sh install` (`docs/head_camera.md`).

| Pad | |
|---|---|
| hold **RT** + left stick | move and strafe (RT is the deadman) |
| hold **RT** + right stick | turn |
| d-pad up / down | speed level: XLeRobot's 0.1 / 0.2 / 0.3 m/s, 30 / 60 / 90 °/s; starts at the slowest |
| **A** | pick and place, one continuous move (with the arm) |
| **B** | abort: the arm holds where it is |
| no pad | hold W A S D to move, Q E to turn |

Sticks are proportional within a level. Level 3 is about the base's ceiling: the wheel
servos top out near `max_raw` (3000 steps/s), about 0.27 m/s straight ahead on 5 cm wheels.

## One Start, arm or not

```
browser (localhost) ──HTTP──▶ serve.py --drive (webui/drive_proxy.py)
                                 ├─ arm plugged in:  scripts/arm.sh head handeye pick, on a pty
                                 │                   camera, arm, head, magnet AND base; /base, /state, /cmd
                                 └─ arm not plugged: scripts/drive_bridge.py over a unix socket
                                                     camera (30 fps) + wheel servos 7/8/9
```

**Start** looks for the arm's board (USB serial `5B91029024`, `ARM_BOARD_SERIAL`). Plugged in:
it asks first (the arm moves to its rest pose as the program starts), then runs `handeye pick`,
the same program as in a terminal (`docs/head_camera.md`). Its key `o` is the one-button pick: find
the package, pick it, carry it to the taught place pose, release, back to rest. Pad A sends it,
B sends a key (any key aborts a move). The base drives through handeye's `/base`, and stops before
any pick starts. Teach the place pose once (`docs/head_camera.md`, "Pick: place pose"); without one
a pick ends holding the package.

Not plugged in: the camera and the wheels only, in the same view. The **Arm** light says which.
Unplug the arm while the pick runs and serve.py quits the pick (its own `x x` quit) and starts the
drive-only bridge in its place, with a banner saying why; the base stays drivable.

**Stop** with the arm running asks you to support it first: its torque goes off and the magnet
lets go (handeye's own quit). A `handeye pick` started in a terminal is used too; Start sees it
answering and doesn't start another.

## Safety

- **Deadman.** Commands go out only while RT (or a key) is held. The base stops 0.3 s after the
  last one (`watchdog_s`, and handeye's `BASE_FRESH_S`), so a closed tab, a dropped link or a
  crashed browser stops the robot. One zero is also sent on release.
- **serve.py dying.** The bridge watches its stdin (EOF: wheels to 0, exit). handeye on its pty
  gets a hang-up, which it treats as Ctrl-C: magnet off, base stopped, torque off.
- **What software can't stop.** An STS3215 keeps its last speed until told otherwise. If the
  wheels' board drops off USB, the bridge keeps reopening it, and reopening writes 0 first. If
  the process driving the wheels is SIGKILLed (OOM killer) mid-drive, nothing is left to send
  that 0. The base's power switch is the e-stop: keep it in reach.
- **Who can drive.** The routes exist only with `--drive`, and answer only a browser on the
  same machine. A request through Tailscale Serve/Funnel or any other proxy is refused, because
  serve.py has no authentication (its docstring).
- **One owner.** A servo board and the D435 take one process at a time. The bridge names the
  process holding one (pid + command) instead of opening it underneath.
- **Pushing it by hand.** With the wheels stopped their torque is off, but each wheel is still a
  ~1:345 gearbox: nudge it slowly, don't shove it. Carry it or drive it to move it far.

## Setup

serve.py needs numpy and PyYAML. On the laptop only the `lerobot` env has both, hence the
command above. `SORTBOTS_LEROBOT_PYTHON` overrides which python runs the bridge.

`configs/drive_base.yaml` holds the wheels' port, ids, geometry, speed levels and the camera.
The wheels are on their own board, found by its USB serial (plugging it in renumbered the arm's
board), with XLeRobot's ids 7/8/9:

```bash
~/miniconda3/envs/lerobot/bin/python scripts/drive_bridge.py scan          # what answers on the bus
~/miniconda3/envs/lerobot/bin/python scripts/drive_bridge.py setup left    # ONE new servo alone: give it its id
~/miniconda3/envs/lerobot/bin/python scripts/drive_bridge.py wheels        # on blocks: each wheel, then a turn
```

`setup` refuses unless exactly one servo answers. A new STS3215 ships as id 1, the same as the
arm's `shoulder_pan` (`head.py setup` has the same check).

## Tests

```bash
/usr/bin/python3 -m pytest tests/drive_bridge_test.py   # the handeye checks need numpy: skipped without it
node webui/tests/operator_test.mjs                       # the "drive" checks
```

No hardware: the bridge runs with its camera off and a port that doesn't exist, handeye's real
HTTP server runs on a free port, and a fake `handeye pick` on a real pty stands in for the arm.
