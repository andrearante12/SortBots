# SO-101 electromagnet end effector

This replaces the follower arm's gripper with a 25 mm holding electromagnet on a
spring-compliant plunger. The stock gripper is the fixed jaw, the moving jaw and
motor 6. A microswitch confirms the magnet has seated flat on the package.

## The magnet: Adafruit 3873

**[Adafruit 3873](https://www.adafruit.com/product/3873)** is a 5 V
electromagnet with 5 kg holding force, in the P25/20 body size (Ø25 × 20 mm).
It costs $9.95, and it's also stocked by DigiKey, The Pi Hut and Core
Electronics.

| | |
|---|---|
| Supply | 5 V DC, 0.3 A (1.5 W). Any 5 V or USB rail can power it, so no 12 V boost converter is needed. |
| Size and weight | Ø25 × 20 mm, Ø12 centre pole, 55.3 g, 280 mm leads |
| Mounting | one tapped hole in the centre of the back. The screw, split washer and flat washer are included. |
| Usable lift | about 0.5–1 kg on a flat mild-steel plate (holding force ÷ 5–10, per Adafruit) |

**Not published, so these numbers are estimates:**
- The thread is M4 and 12 mm deep. That's the spec of the JF-XP2520, which has
  the same body.
- The lead stub sits about 4 mm from the back face and is Ø3.5. I read that
  off Adafruit's product photos.

Measure all three with calipers when the part arrives, and correct them in
`params.py`.

**How it attaches.** The leads leave through the magnet's **side**, near the
back, inside a stiff heat-shrink stub. They don't come out of the back face.
- **Keeper:** the plunger has a 50° arc of skirt under it, with a slot open at
  the bottom. The stub slides into that slot as you push the magnet up into
  the plunger. A single centre screw can't stop the magnet turning, so the
  keeper is what fixes its orientation. It also takes the strain off the stub.
- **Bolt:** the counterbore is sized for the included screw *with* both
  washers. The build prints the allowed screw length, which is currently
  9.6–16.6 mm under the head.
- **Rim notch:** at full compression the stub rises into a notch in the cup
  rim. From there the leads run up the *outside* of the cup to the zip-tie
  lug, so they never enter the cup.

The coil needs a flyback diode. Drive it from a logic-level MOSFET or a motor
driver.

```
       STS3215 motor 5 (wrist roll)
       ════╤═══ horn ═══╤════         z = 0    horn face (4x M3 on 9.9 mm square, same as stock)
           │  Ø24 neck  │                      clears the motor-5 bracket (reaches z=5.3)
   ┌───────┴────────────┴───────┐     z = 6.5
   │ roof           [switch]    │  ← adapter_body (printed)
   │  ║standoff║ spring         │         z_standoff: plunger hard-stops here
   ├──╨─────────────────────────┤  ← plunger (printed), 3 mm travel
   └──▀──────┬──────────┬────▀──┘         ▀ = M3 guide-screw heads (extend stop)
             │  magnet  │
             └──────────┘                 z ≈ 47.1  TCP / magnet face
```

## Setup (once)

```bash
hardware/setup_cad.sh            # venv + CadQuery + reference STEPs (+20 MB arm); exit 2 = FreeCAD missing
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
4. On the whole arm (about 1 min). This builds `arm_context.step` and sweeps
   the full wrist-roll range for collisions with the arm. It exits 3 if
   anything hits.
   ```bash
   hardware/.venv/bin/python hardware/magnet_effector/build.py --arm   # or: make cad-arm
   freecad hardware/magnet_effector/build/arm_context.step &           # or: make cad-open-arm
   ```
5. Re-run the geometry checks (about 45 s, most of it loading the arm):
   ```bash
   hardware/.venv/bin/python -m pytest tests/cad_test.py           # or: make cad-test
   ```

| output (`build/`, gitignored) | what |
|---|---|
| `adapter_body.step/.stl`, `plunger.step/.stl` | the two printed parts |
| `assembly.step` | the parts plus placeholders (magnet, screws, switch) and motor 5, at rest |
| `fitcheck.step` | the same, with the stock wrist overlaid in translucent red; the horn holes must line up |
| `arm_context.step` | (`--arm`) the whole SO-101 follower with the stock gripper removed and the effector on motor 5 |

### The whole-arm model (`arm.py`)

`arm.py` loads upstream's full SO-101 assembly and finds the stock fixed jaw
in it. It then registers our frame onto that jaw by matching volume and
inertia. Every vertex lands within 0.000 mm, and the build refuses anything
over 0.01. No placement numbers are typed in by hand.

After registering, it splits the arm's parts three ways:
- **gripper:** everything centred past motor 5's horn face (the jaws,
  motor 6, its screws, nuts and cable). These are dropped.
- **horn:** the 4 horn screws and the centre screw. These are kept, and they
  turn with the effector.
- **arm:** everything else.

The roll sweep spins our parts through 360° against the **arm** group. That
sweep is why the neck exists: a full-width cup hits the `Wrist_Roll_Pitch`
bracket at most roll angles, and `test_roll_sweep_catches_a_full_width_cup`
keeps that check honest.

### Working in FreeCAD

Every output uses the same tree, built so you can move whole groups at once:

```
magnet_effector              origin = centre of motor 5's horn, W axis = out along the wrist
 |- printed_parts            what you 3D print: adapter_body, plunger
 |- magnet_Adafruit_3873     what you buy: body, potting, lead_stub, leads
 |- hardware                 guide screws, springs, M4 magnet screw, microswitch, horn screws
```

In `arm_context.step` this sits next to `SO101_arm_stock`, the untouched
arm. To pull it apart:
1. Right-click a group, choose **Transform**, set Coordinate system to
   **Local**, and type a value into **W**.
2. A suggested exploded view: `hardware` at +20, `printed_parts` at +45 and
   `magnet_Adafruit_3873` at +90.
3. Moving `magnet_effector` itself takes the whole effector off the arm.

**One-click exploded view.** `explode.FCMacro` animates the parts apart along
the wrist, and running it again puts them back exactly where they were.
Ctrl+Z undoes a toggle. Setup:
1. Link it into FreeCAD's macro folder, once (the snap's folder is shown):
   ```bash
   ln -sf "$PWD/hardware/magnet_effector/explode.FCMacro" ~/snap/freecad/common/sortbots_explode.FCMacro
   ```
2. Run it from Macro → Macros… → `sortbots_explode` → Execute.
3. For a toolbar button or hotkey, use Tools → Customize → Macros.

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
- Want to try a different magnet? Change the `Magnet` fields: `d`, `h`, the
  lead stub (`lead_z_from_back`, `lead_stub_d`), and `mass_g`. 20 mm and
  34 mm are both covered by the tests.

## What to verify before printing

Everything marked **VERIFY** in `params.py` is a datasheet guess. Check these
against the actual parts in hand:

- **Magnet (Adafruit 3873):** the thread size and depth, and the lead stub's
  height from the back face and its diameter. Adafruit doesn't publish these;
  see above.
- **Switch:** body dims, hole pitch, **OP** (actuator height at the operating
  point) and **OT** (overtravel). The plunger hard-stops at full stroke, so
  the switch must tolerate `travel - trip` of overtravel. That's why it trips
  late (2.6 of 3.0 mm): it reports *seated*, not just *touched*. A
  hinge-lever switch gives much more OT if you want an earlier trip.
- **Springs:** OD/ID against the M3 screw, and free length.

## Printing and assembly

- **PETG**, 0.2 mm layers, 6 perimeters (`Print.wall = 2.4`).
- **Orientation:**
  - adapter_body: horn face down. The step from the Ø24 neck out to the Ø45
    cup is a flat ~10 mm overhang, so enable **supports on build plate only**
    (they touch nothing but the outside of the step). The floor bridges the Ø20
    horn pilot recess, which PETG handles fine.
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
3. Push the magnet up into the plunger with its lead stub in the keeper slot.
   Bolt it from the top with the included M4 screw and both washers.
4. Put the springs in the standoff pockets. Fit the plunger, then the 3× M3
   guide screws from below.
5. Run the magnet leads up the outside of the cup from the rim notch. Bring
   the switch wires out of the cable slot. Zip-tie both to the lug.

## Not here yet

The URDF magnet link (use the printed TCP), the LeRobot follower subclass, the
MOSFET driver and the slip ring are all still to do.
