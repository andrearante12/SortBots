# Drive view: the D435 live and the omni base on a gamepad

The operator view's **Drive** view shows the D435 live and drives the omni base with a
gamepad, on the machine the robot is plugged into (the laptop today). It needs no ROS:

```
browser (localhost) ──HTTP──▶ serve.py --drive ──unix socket──▶ scripts/drive_bridge.py
                                                                  (lerobot env)
                                                                  ├─ D435 colour → JPEG
                                                                  └─ wheel servos 7/8/9, velocity mode
```

```bash
~/miniconda3/envs/lerobot/bin/python webui/serve.py --drive   # then open http://localhost:8081/
```

Pick **Drive** next to Map / 3D cloud, press **Connect**, press any button on the pad.

**Pick and place from the pad.** Start the pick first, in a terminal, then open the Drive view:

```bash
scripts/arm.sh head handeye pick
```

`handeye pick` owns the camera, the arm, the magnet and the base, so the Drive view goes through
it (no Connect needed): **A** is its one-button pick and place (find the package, pick it, carry it
to the taught place spot, release, back to rest, one continuous move), **B** aborts and holds the
arm where it is, and RT + sticks still drive the base between picks. The base stops before any
pick starts. Teach the place spot once in handeye's own page (Save place here); without one a
pick ends holding the package. Without `handeye pick`, Connect starts the drive-only bridge and A
only says that the pick isn't running.

- **Pad:** hold **RT** to drive (deadman), left stick moves and strafes, right stick turns,
  d-pad up/down picks one of XLeRobot's three speeds (0.1 / 0.2 / 0.3 m/s). No pad: hold
  W A S D / Q E.
- **Stopping:** commands go out only while RT (or a key) is held. The bridge stops the wheels
  0.3 s after the last one, so a closed tab, a dropped link or a crashed browser stops the
  robot. If serve.py dies, the bridge sees its stdin close and stops too. **Disconnect**
  sets the wheels free to roll and releases the bus and the camera.
- **What software can't stop:** an STS3215 keeps its last speed until told otherwise. If
  the board drops off USB, the bridge keeps reopening it, and reopening writes 0 first. If
  the bridge itself is SIGKILLed (OOM killer) mid-drive, nothing is left to send that 0.
  The base's power switch is the e-stop, so keep it in reach.
- **Who can drive:** the routes exist only with `--drive`, and answer only a browser on the
  same machine. A request that came through Tailscale Serve/Funnel or another proxy is
  refused, because serve.py has no authentication (its docstring).
- **One owner:** the camera and the wheels' board are the ones `handeye pick` opens (it
  backs the base off the table to place). Connect names the process holding either one
  (e.g. `head.py handeye`) instead of opening it underneath. Stop that process first.
  Each half works on its own, so a camera with no wheels yet is still a camera.
- **Python:** serve.py needs numpy and PyYAML. On the laptop only the `lerobot` env has
  both, hence the command above. `SORTBOTS_LEROBOT_PYTHON` overrides which python runs the
  bridge.

Wheel setup: `configs/drive_base.yaml` holds the port, ids, geometry and speed levels. The
wheels are on their own board, found by its USB serial (plugging it in renumbered the arm's
board). Their ids are XLeRobot's 7/8/9:

```bash
~/miniconda3/envs/lerobot/bin/python scripts/drive_bridge.py scan          # what answers on the bus
~/miniconda3/envs/lerobot/bin/python scripts/drive_bridge.py setup left    # ONE new servo alone: give it its id
~/miniconda3/envs/lerobot/bin/python scripts/drive_bridge.py wheels        # on blocks: each wheel, then a turn
```

`setup` refuses unless exactly one servo answers. A new STS3215 ships as id 1, the same as
the arm's `shoulder_pan` (`head.py setup` has the same check).
