"""Every tweakable number for the SO-101 electromagnet end effector, in mm / g.

Change a value here, run `make cad`, reopen build/assembly.step in FreeCAD.

Frame: +Z is the wrist-roll axis pointing AWAY from the servo. z=0 is the face
that seats on the STS3215 output horn (motor 5); the magnet face is the most
positive z. Same orientation as the stock Wrist_Roll_Follower_SO101.step, so
the reference part overlays directly (see model.reference_wrist()).

Layout (looking down +Z): guide pins at 90/210/330 deg, microswitch at 30 deg,
cable exit at 150 deg — the three free sectors between pins.
"""
from dataclasses import dataclass, field


@dataclass(frozen=True)
class Horn:
    # MEASURED from reference/Wrist_Roll_Follower_SO101.step (SO-ARM100 @5f6d2b8):
    # 4x r=1.6 cylinders at x=+-4.95, y=-5.17/+4.73 -> 9.9 mm square centred on
    # the roll axis (which sits at y=-0.22 in that file); matches the 4x r=1.25
    # tapped holes on the STS3215 horn (reference/STS3215_03a.step, 7.55..17.45
    # around the shaft at x=12.5). Don't eyeball these: tests/cad_test.py
    # re-measures the stock part live and fails if our holes drift from it.
    hole_square: float = 9.9
    hole_d: float = 3.2          # M3 clearance, same as stock
    center_hole_d: float = 6.0   # horn centre screw. Stock is 5.4, but the SO-101
                                 # assembly's own centre screw head is 5.6 and
                                 # overlaps the stock part by 1.4 mm3 — open it up.
    pilot_d: float = 20.0        # stock recess the horn disc locates in (r=10.0)
    pilot_depth: float = 1.0     # stock: z -0.05 .. 0.95; the horn disc face
                                 # (z=18.7 in STS3215_03a.step) seats on its floor
    floor_t: float = 3.0         # stock: z 0.95 .. 3.95
    screw_head_h: float = 3.0    # M3 SHCS head; heads sit on the floor's inner face
    head_cbore_d: float = 6.4    # head pockets through the neck, driver goes in via the cup
    # The neck is NOT cosmetic. The Wrist_Roll_Pitch bracket that holds motor 5
    # reaches 5.3 mm past the horn face at r >= 12.6 (measured in the SO-101
    # assembly, see arm.py), which is why the stock part is a Ø24 boss for its
    # first 6 mm. A full-width cup from z=0 hits that bracket at most roll
    # angles (~500 mm3 overlap, found 2026-10-05); test_clears_wrist_bracket
    # sweeps the full roll range.
    neck_d: float = 24.0
    neck_len: float = 6.5


@dataclass(frozen=True)
class Magnet:
    # Adafruit 3873: 5 V DC, 0.3 A, 5 kg holding force, P25/20 body, $9.95.
    # https://www.adafruit.com/product/3873  (also DigiKey 1528-3873-ND, The Pi
    # Hut, Core Electronics). Body dims, pole and mass are Adafruit's published
    # numbers. Thread and lead position are NOT published: they're from
    # Adafruit's product photos plus the same-body JF-XP2520 spec (M4, 12 deep).
    # VERIFY those with calipers when it arrives.
    # Usable lift is roughly holding force / 5..10, so ~0.5-1 kg on a flat
    # mild-steel plate.
    d: float = 25.0
    h: float = 20.0
    pole_d: float = 12.0          # centre pole on the face ("center diameter")
    potting_od: float = 21.5      # cosmetic: dark ring around the pole (product photo 4)
    tap: str = "M4"               # VERIFY
    thread_depth: float = 12.0    # VERIFY
    bolt_clear_d: float = 4.5     # M4 clearance through the plunger
    # The included screw is a pan head with a split lock washer AND a flat
    # washer (product photo 2), so the counterbore is sized to the flat
    # washer (M4 OD 9.0), not the head.
    bolt_head_d: float = 9.5
    bolt_head_h: float = 5.0      # pan head ~2.7 + split ~1.3 + flat 0.8, rounded up
    washer_stack: float = 2.1     # split + flat washer: eats into the screw's usable length
    # The leads leave through the SIDE, near the back, inside a stiff
    # heat-shrink stub, not out of the back face (photos 1 and 4). A single
    # centre screw can't hold rotation, so the plunger grows a keeper arc with
    # a slot that captures the stub: it keys the orientation and strain-relieves
    # the stub.
    lead_angle: float = 150.0     # deg; a free sector between guide pins, next to the zip-tie lug
    lead_z_from_back: float = 4.0  # VERIFY: stub centre, measured from the back face
    lead_stub_d: float = 3.5      # VERIFY: heat-shrink OD
    lead_stub_len: float = 8.0    # how far the stiff part sticks out radially
    keeper_wall: float = 1.6
    keeper_span: float = 50.0     # deg of arc; must stay clear of the guide-screw heads
    locate_depth: float = 1.0     # shallow recess in the plunger that centres the magnet
    mass_g: float = 55.3          # Adafruit's published weight
    current_a: float = 0.3
    voltage: float = 5.0


@dataclass(frozen=True)
class Compliance:
    travel: float = 3.0           # plunger stroke; hard-stops on the standoffs
    pin_d: float = 3.0            # M3 guide screws (shoulder screws if you can get them)
    pin_slide_clear: float = 0.4  # plunger slides on the screw shank
    pin_head_d: float = 5.5       # M3 SHCS head — this is the extend stop, under the plunger
    pin_angles: tuple = (90.0, 210.0, 330.0)
    tap_d: float = 2.5            # M3 self-tap into the printed standoff (or 4.0 for a heat-set insert)
    spring_od: float = 5.5        # VERIFY against the spring you buy; ID must clear pin_d
    spring_free_len: float = 10.0
    spring_preload: float = 1.0   # compression at full extension, so the plunger never rattles
    spring_seat_plunger: float = 1.5  # spring pocket depth in the plunger top
    slide_clear: float = 0.3      # radial, plunger in the cup bore


@dataclass(frozen=True)
class Switch:
    # Omron D2F-class subminiature. VERIFY body dims, OP and OT on the datasheet.
    # The plunger hard-stops on the standoffs at full travel, so the switch must
    # survive (travel - trip) of overtravel. Subminiature switches only allow
    # ~0.5 mm OT, so the trip point sits LATE in the stroke: the switch confirms
    # the magnet has SEATED flat (which is when it actually holds), not just
    # touched. build.py prints the margin; the test fails if it's negative.
    l: float = 12.8               # tangential
    w: float = 6.5                # radial
    h: float = 5.8                # along Z; actuator points +Z at the plunger
    op: float = 0.8               # body face -> actuator tip at the operating point
    ot: float = 0.5               # overtravel past OP the switch tolerates (must be < op)
    hole_pitch: float = 6.5
    hole_from_act: float = 2.9    # mounting-hole centre -> actuator-side body face
    screw_d: float = 2.2          # M2 clearance through the outer wall
    trip: float = 2.6             # plunger compression at which the switch closes
    adjust: float = 1.5           # +- slot length so the trip point can be tuned on the bench
    angle: float = 30.0


@dataclass(frozen=True)
class Print:
    wall: float = 2.4             # 6 perimeters at 0.4 mm
    clearance: float = 0.2        # generic fit clearance
    fillet: float = 0.8
    density_g_cm3: float = 1.27   # PETG
    infill_factor: float = 0.6    # rough solid-equivalent for the mass estimate


@dataclass(frozen=True)
class Params:
    horn: Horn = field(default_factory=Horn)
    magnet: Magnet = field(default_factory=Magnet)
    comp: Compliance = field(default_factory=Compliance)
    switch: Switch = field(default_factory=Switch)
    pr: Print = field(default_factory=Print)
    cable_slot_w: float = 6.0
    cable_angle: float = 150.0
    ziptie_w: float = 3.0         # strain-relief slot pair beside the cable exit
    # Removed with the stock gripper, for the mass comparison in build.py.
    removed_servo_g: float = 55.0  # STS3215 (motor 6)

    # ---- derived (don't edit; change the inputs above) ---------------------
    @property
    def floor_top(self) -> float:
        """Inner face of the floor (where horn screw heads sit)."""
        return self.horn.pilot_depth + self.horn.floor_t

    @property
    def cavity_top(self) -> float:
        """Inner roof of the cup: neck plus a wall-thick roof over the step."""
        return self.horn.neck_len + self.pr.wall

    @property
    def pin_circle_r(self) -> float:
        # Screw heads sit under the plunger beside the magnet, so they must clear it.
        return self.magnet.d / 2 + self.comp.pin_head_d / 2 + 1.0

    @property
    def plunger_d(self) -> float:
        return 2 * (self.pin_circle_r + self.comp.pin_head_d / 2 + 1.0)

    @property
    def plunger_floor(self) -> float:
        """Material the magnet bolt clamps: under the counterbore, above the magnet recess."""
        return 2.5

    @property
    def plunger_t(self) -> float:
        # Earlier versions had head_h + 2 and forgot the locate recess also
        # eats from the bottom, which left a 1 mm floor (2026-10-05).
        m = self.magnet
        return m.bolt_head_h + self.plunger_floor + m.locate_depth

    @property
    def magnet_bolt_len(self) -> tuple:
        """(min, max) under-head length for the magnet screw: >=5 mm of thread
        engaged, never bottoming in the tapped hole."""
        m = self.magnet
        base = m.washer_stack + self.plunger_floor
        return base + 5.0, base + m.thread_depth

    @property
    def bore_d(self) -> float:
        return self.plunger_d + 2 * self.comp.slide_clear

    @property
    def body_d(self) -> float:
        return self.bore_d + 2 * self.pr.wall

    @property
    def standoff_d(self) -> float:
        return self.comp.spring_od + 2 * self.pr.wall

    @property
    def spring_seat_standoff(self) -> float:
        c = self.comp
        return c.spring_free_len - c.spring_preload - c.travel - c.spring_seat_plunger

    @property
    def z_standoff(self) -> float:
        """Bottom of the standoffs = plunger top at FULL compression."""
        s, c = self.switch, self.comp
        # The switch body must fit between the roof and the plunger with the
        # full upward adjust still available.
        need_switch = self.cavity_top + s.h + s.op + s.trip + s.adjust - c.travel
        need_spring = self.cavity_top + self.spring_seat_standoff + 1.0
        return max(need_switch, need_spring)

    @property
    def z_plunger_top(self) -> float:
        """Plunger top at rest (fully extended)."""
        return self.z_standoff + self.comp.travel

    @property
    def body_len(self) -> float:
        # Cup is flush with the plunger bottom at rest, so the plunger stays
        # fully guided over its whole stroke.
        return self.z_plunger_top + self.plunger_t

    @property
    def keeper_h(self) -> float:
        """Keeper arc height below the plunger: covers the stub plus a lip."""
        m = self.magnet
        return m.lead_z_from_back - m.locate_depth + m.lead_stub_d / 2 + 1.5

    @property
    def z_magnet_face(self) -> float:
        """TCP at rest, measured from the horn face along +Z."""
        return self.body_len - self.magnet.locate_depth + self.magnet.h

    @property
    def z_switch_body(self) -> float:
        """Switch body face nearest the plunger (actuator side), nominal slot centre."""
        return self.z_plunger_top - self.switch.trip - self.switch.op

    @property
    def switch_ot_margin(self) -> float:
        return self.switch.ot - (self.comp.travel - self.switch.trip)


DEFAULT = Params()
