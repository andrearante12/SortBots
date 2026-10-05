# SO-101 electromagnet end effector

This replaces the follower arm's gripper with a 25 mm holding electromagnet on a
spring-compliant plunger. The stock gripper is the fixed jaw, the moving jaw and
motor 6. A microswitch confirms the magnet has seated flat on the package.

```
       STS3215 motor 5 (wrist roll)
       ════╤═══ horn ═══╤════         z = 0   horn face (4x M3 on 9.9 mm square, same as stock)
   ┌───────┴────────────┴───────┐
   │ floor          [switch]    │  ← adapter_body (printed)
   │  ║standoff║ spring         │         z_standoff: plunger hard-stops here
   ├──╨────────────────────────-┤  ← plunger (printed), 3 mm travel
   └──▀──────┬──────────┬────▀──┘         ▀ = M3 guide-screw heads (extend stop)
             │  magnet  │
             └──────────┘                 z ≈ 39.7  TCP / magnet face
```

## Setup (once)

```bash
hardware/setup_cad.sh            # venv + CadQuery + reference STEPs; exit 2 = FreeCAD missing
sudo snap install freecad        # Ubuntu 26.04 has no apt package
```

`hardware/.venv` is its own environment, so never install CadQuery into the
sim, ROS or lerobot environments. It's built from the system `python3`.
CadQuery's OCP has 3.14 wheels, so conda isn't needed.

## The loop: change a number, rebuild, look

1. Edit **`params.py`**. Every dimension lives there, grouped as
   `Horn` / `Magnet` / `Compliance` / `Switch` / `Print`. The derived
   properties at the bottom (`z_standoff`, `body_len`, …) recompute
   themselves, so don't edit those.
2. Rebuild (about 4 s):
   ```bash
   hardware/.venv/bin/python hardware/magnet_effector/build.py     # or: make cad
   ```
   It prints the derived values you need elsewhere: the TCP offset for the
   URDF, the mass vs. the removed stock parts, the switch overtravel margin
   and the BOM.
3. Open it:
   ```bash
   freecad hardware/magnet_effector/build/fitcheck.step &          # or: make cad-open
   ```
4. Re-run the geometry checks:
   ```bash
   hardware/.venv/bin/python -m pytest tests/cad_test.py           # or: make cad-test
   ```

| output (`build/`, gitignored) | what |
|---|---|
| `adapter_body.step/.stl`, `plunger.step/.stl` | the two printed parts |
| `assembly.step` | the parts plus placeholders (magnet, screws, switch) and motor 5, at rest |
| `fitcheck.step` | the same, with the stock wrist overlaid in translucent red; the horn holes must line up |

### Working in FreeCAD

- STEP imports are **dead solids**: FreeCAD doesn't know they came from
  `params.py`. Use FreeCAD to *look*:
  - section with View → Clipping plane
  - measure with the Measure toolbar
  - check interference with Part → Check geometry, or Part → Boolean → Common
    on two bodies
  - make the stock overlay translucent with V,T
  
  Make *changes* in `params.py`, then rebuild and reopen.
- If you'd rather sketch a feature by hand (e.g. a quick fillet), do it on a
  copy and port the number back to `params.py`. Otherwise the next rebuild
  loses it.
- Want to try a different magnet? Change `Magnet.d`, `h`, `lead_r` and
  `mass_g`. 20 mm and 34 mm are both covered by the tests.

## What to verify before printing

Everything marked **VERIFY** in `params.py` is a datasheet guess. Check these
against the actual parts in hand:

- **Magnet:** diameter, height, thread size, *where the leads exit*
  (`lead_r`, `lead_angle`), and mass.
- **Switch:** body dims, hole pitch, **OP** (actuator height at the operating
  point) and **OT** (overtravel). The plunger hard-stops at full stroke, so
  the switch must tolerate `travel - trip` of overtravel. That's why it trips
  late (2.6 of 3.0 mm): it reports *seated*, not just *touched*. A
  hinge-lever switch gives much more OT if you want an earlier trip.
- **Springs:** OD/ID against the M3 screw, and free length.

## Printing and assembly

- **PETG**, 0.2 mm layers, 6 perimeters (`Print.wall = 2.4`).
- **Orientation:**
  - adapter_body: horn face down. The floor prints flat, and the cup needs no
    supports.
  - plunger: magnet face down.
- The M3 guide-screw holes are tapped by self-tapping into the plastic
  (`tap_d = 2.5`). For heat-set inserts, set `tap_d` to the insert's hole size.

**Print the adapter alone first** and test-fit it on the motor 5 horn before
printing the plunger.

Assembly order:
1. Bolt the adapter to the horn: 4× M3 from inside the cup, reusing the stock
   screws.
2. Fit the switch: M2 screws from outside through the wall slots, nuts on the
   inside. Slide it to set the trip point.
3. Bolt the magnet to the plunger with M4 from the top. Thread the leads up
   through the lead hole.
4. Put the springs in the standoff pockets. Fit the plunger, then the 3× M3
   guide screws from below.
5. Route the leads out the cable slot and zip-tie them to the lug.

## Not here yet

The URDF magnet link (use the printed TCP), the LeRobot follower subclass, the
MOSFET driver and the slip ring are all still to do.
