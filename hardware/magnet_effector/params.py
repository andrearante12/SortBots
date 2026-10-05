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
    center_hole_d: float = 5.4   # access to the horn's centre screw
    pilot_d: float = 20.0        # stock recess the horn disc locates in (r=10.0)
    pilot_depth: float = 1.0     # stock: z -0.05 .. 0.95; the horn disc face
                                 # (z=18.7 in STS3215_03a.step) seats on its floor
    floor_t: float = 3.0         # stock: z 0.95 .. 3.95
    screw_head_h: float = 3.0    # M3 SHCS head; heads sit on the floor's inner face


@dataclass(frozen=True)
class Magnet:
    # Generic 25 mm "holding electromagnet" (P25/20-style). VERIFY against the
    # part you actually buy — lead position in particular varies by vendor.
    d: float = 25.0
    h: float = 20.0
    tap: str = "M4"
    bolt_clear_d: float = 4.5     # M4 clearance through the plunger
    bolt_head_d: float = 7.5      # M4 SHCS head is 7.0; +0.5 for the counterbore
    bolt_head_h: float = 4.0
    lead_r: float = 7.0           # radius where the leads exit the magnet's back
    lead_angle: float = 150.0     # deg; aligned with the cable exit slot
    lead_hole_d: float = 5.0
    locate_depth: float = 1.0     # shallow recess in the plunger that centres the magnet
    mass_g: float = 60.0          # VERIFY: weigh yours; used only for the printed estimate


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
    def pin_circle_r(self) -> float:
        # Screw heads sit under the plunger beside the magnet, so they must clear it.
        return self.magnet.d / 2 + self.comp.pin_head_d / 2 + 1.0

    @property
    def plunger_d(self) -> float:
        return 2 * (self.pin_circle_r + self.comp.pin_head_d / 2 + 1.0)

    @property
    def plunger_t(self) -> float:
        return self.magnet.bolt_head_h + 2.0

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
        # The switch body must fit between the floor and the plunger with the
        # full upward adjust still available.
        need_switch = self.floor_top + s.h + s.op + s.trip + s.adjust - c.travel
        need_heads = self.floor_top + self.horn.screw_head_h + 1.0
        need_spring = self.floor_top + self.spring_seat_standoff + 1.0
        return max(need_switch, need_heads, need_spring)

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
