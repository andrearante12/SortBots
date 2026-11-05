"""The magnet end effector, as the sim models it: a capture rule and a snap.

hardware/magnet_effector/ is the real part (Adafruit 3873, 25 mm, ~5 kg holding
force on a spring plunger). In the sim there is no contact, no compliance and no
slip: when the magnet is energized and its face is within CAPTURE_M of a
package's magnet plate, pointing at it within CAPTURE_DEG, the package is
SNAPPED to the magnet (face centre on the TCP, face normal anti-parallel to the
approach) and rides the arm until the magnet is de-energized, when it drops
straight down onto whatever is under it. That is the whole grasp model.

Pure python + math, no numpy, because it is imported from four places:
Isaac's venv (scripts/spawn_warehouse.py applies it every physics step), the
pick controller (nodes/pick_planner.py aims with it), the metrics, and the
offline tests. Keep it that way.

FRAMES. Everything here is the `map` frame (== Isaac's world frame) unless a
name says `base`. A package's pose is {x, y, z, yaw}: x, y is its centre, **z is
the height of its BOTTOM** (so a package on a 0.80 m deck has z=0.80), yaw
rotates it about +z. A face is one of simtest.world.FACES; the magnet plate sits
centred on that face, and `normal` is the plate's OUTWARD normal. The magnet
touching it therefore approaches along -normal.

CONSTANTS. The arm numbers come from third_party/XLeRobot/.../xlerobot.urdf
(arm 1, the one the sim drives) by forward kinematics over its joint limits,
which are also what spawn_warehouse clamps to: so in the sim they are the truth,
whatever the real SO-101 can do.
"""
from __future__ import annotations

import math
from typing import Callable

from . import geometry, world

# -- the effector -------------------------------------------------------------
# TCP = the magnet face, TCP_OFFSET_M from the `Fixed_Jaw` link origin. Fixed_Jaw
# is the link motor 5 (wrist roll) turns, so it is where the real effector's horn
# mounts. The roll axis is Fixed_Jaw -Y (URDF `Wrist_Roll` axis 0 -1 0; the stock
# jaw tip sits at y=-0.097), and the magnet face is z_magnet_face = 47.1 mm from
# the horn face along it (hardware/magnet_effector/params.py, `Params.z_magnet_face`,
# at rest, before the plunger's 3 mm of travel). tests/magnet_test.py recomputes
# 47.1 mm from params.py (a pure dataclass, no cadquery) so the two cannot drift.
TCP_OFFSET_M = (0.0, -0.0471, 0.0)
# The unit vector the magnet points along, in the Fixed_Jaw frame.
APPROACH_LOCAL = (0.0, -1.0, 0.0)

# -- the arm, in base_link ------------------------------------------------------
BASE_Z_M = 0.05                    # base_link sits this far above the floor (spawn z)
# `Rotation` joint origin in base_link: arm_base_joint (-0.135, -0.133, 0.760)
# + Rotation (0, -0.0452, 0.0165) rotated by arm_base_joint's yaw of pi/2.
SHOULDER_XYZ_BASE = (-0.090, -0.133, 0.776)
# At Rotation=0 the approach is base +X. The arm is a single vertical plane
# through the shoulder, so the approach azimuth is always radial from it: that is
# why "any orientation" is a base-placement problem (dock_pose), not an arm one.
# Where dock_pose puts the plate, radially from the shoulder. The plate is then
# 0.27 - 0.09 = 0.18 m ahead of base_link, which is as close as the chassis lets
# it be: base_link's collision is a thin frame (urdf, z 0.685-0.775 above base
# origin) whose front wall sits 0.153 m ahead of the origin, and a deck stands
# full height to the floor. Worlds overhang the package 2 cm past the deck edge
# so the wall stops ~4 cm short of the deck face.
REACH_NOM_M = 0.27
REACH_MAX_M = 0.33                 # horizontal reach at deck height, horizontal approach
CHASSIS_HALF_M = (0.158, 0.203)    # the frame's collision footprint about base_link (x, y)
TCP_Z_RANGE_BASE = (0.60, 1.24)    # TCP height range over the joint limits, base frame

# -- the capture rule ------------------------------------------------------------
CAPTURE_M = 0.03        # face centre to TCP
CAPTURE_DEG = 30.0      # angle between the approach and the plate's inward normal
STANDOFF_M = 0.08       # pre-approach distance out along the plate's normal

FACE_LOCAL = {"+x": (1.0, 0.0, 0.0), "-x": (-1.0, 0.0, 0.0),
              "+y": (0.0, 1.0, 0.0), "-y": (0.0, -1.0, 0.0), "top": (0.0, 0.0, 1.0)}


def _rot(yaw: float, x: float, y: float) -> tuple[float, float]:
    c, s = math.cos(yaw), math.sin(yaw)
    return (c * x - s * y, s * x + c * y)


def _dot(a, b) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _norm(a) -> float:
    return math.sqrt(_dot(a, a))


def _unit(a) -> tuple[float, float, float]:
    n = _norm(a)
    return (a[0] / n, a[1] / n, a[2] / n)


def _face_offset(size, face: str) -> tuple[float, float, float]:
    """Plate centre relative to (package centre x, y, bottom z), package frame."""
    sx, sy, sz = size
    return {"+x": (sx / 2, 0.0, sz / 2), "-x": (-sx / 2, 0.0, sz / 2),
            "+y": (0.0, sy / 2, sz / 2), "-y": (0.0, -sy / 2, sz / 2),
            "top": (0.0, 0.0, sz)}[face]


def magnet_face_pose(pose: dict, size, face: str) -> tuple[tuple, tuple]:
    """(plate centre, outward normal) in the map frame for a package at `pose`."""
    ox, oy, oz = _face_offset(size, face)
    rx, ry = _rot(pose["yaw"], ox, oy)
    nx, ny, nz = FACE_LOCAL[face]
    nrx, nry = _rot(pose["yaw"], nx, ny)
    return ((pose["x"] + rx, pose["y"] + ry, pose["z"] + oz), (nrx, nry, nz))


def capture(tcp, approach, centre, normal) -> tuple[bool, float, float]:
    """(captured, distance_m, angle_deg): would an energized magnet at `tcp`,
    pointing along `approach`, grab the plate at `centre` facing `normal`?"""
    dist = math.dist(tcp, centre)
    a = _unit(approach)
    cosang = max(-1.0, min(1.0, -_dot(a, normal)))
    ang = math.degrees(math.acos(cosang))
    return (dist <= CAPTURE_M and ang <= CAPTURE_DEG, dist, ang)


def held_pose(tcp, approach, size, face: str, prev_yaw: float) -> dict:
    """Where a captured package sits: plate centre on the TCP, plate facing back
    along the approach. The package stays upright: a tilted approach tilts the
    ARM, not the box (no contact model to hold it tilted), so a side face is
    yawed to the approach's horizontal part and a top face keeps its yaw."""
    ox, oy, oz = _face_offset(size, face)
    if face == "top":
        return {"x": tcp[0], "y": tcp[1], "z": tcp[2] - oz, "yaw": prev_yaw}
    hx, hy = -approach[0], -approach[1]
    if math.hypot(hx, hy) < 1e-3:       # pointing straight up/down: nothing to yaw to
        yaw = prev_yaw
    else:
        fx, fy, _ = FACE_LOCAL[face]
        yaw = math.atan2(hy, hx) - math.atan2(fy, fx)
    rx, ry = _rot(yaw, ox, oy)
    return {"x": tcp[0] - rx, "y": tcp[1] - ry, "z": tcp[2] - oz, "yaw": yaw}


def nearest(tcp, approach, items) -> dict | None:
    """The closest package to the TCP, by plate-centre distance.

    items: iterable of (id, pose, size, face). Returns {id, dist_m, angle_deg}."""
    best = None
    for pid, pose, size, face in items:
        centre, normal = magnet_face_pose(pose, size, face)
        _, dist, ang = capture(tcp, approach, centre, normal)
        if best is None or dist < best["dist_m"]:
            best = {"id": pid, "dist_m": round(dist, 4), "angle_deg": round(ang, 2)}
    return best


# -- base placement ---------------------------------------------------------------

def dock_pose(face_centre, normal, *, reach: float = REACH_NOM_M,
              fallback_yaw: float = 0.0) -> tuple[float, float, float]:
    """Where the BASE (x, y, yaw) must stand so the arm, at Rotation=0, reaches
    the plate head-on: heading along -normal, the plate centre `reach` from the
    shoulder along the arm's radial line. A top face has no horizontal normal to
    face, so it keeps `fallback_yaw`."""
    if math.hypot(normal[0], normal[1]) < 1e-3:
        yaw = fallback_yaw
    else:
        yaw = math.atan2(-normal[1], -normal[0])
    ox, oy = _rot(yaw, SHOULDER_XYZ_BASE[0] + reach, SHOULDER_XYZ_BASE[1])
    return (face_centre[0] - ox, face_centre[1] - oy, yaw)


def to_base(base: tuple, p) -> tuple[float, float, float]:
    """A map-frame point in base_link, for a base at (x, y, yaw)."""
    dx, dy = p[0] - base[0], p[1] - base[1]
    c, s = math.cos(base[2]), math.sin(base[2])
    return (c * dx + s * dy, -s * dx + c * dy, p[2] - BASE_Z_M)


def to_map(base: tuple, p) -> tuple[float, float, float]:
    x, y = _rot(base[2], p[0], p[1])
    return (base[0] + x, base[1] + y, p[2] + BASE_Z_M)


def dir_to_base(base: tuple, v) -> tuple[float, float, float]:
    c, s = math.cos(base[2]), math.sin(base[2])
    return (c * v[0] + s * v[1], -s * v[0] + c * v[1], v[2])


def dir_to_map(base: tuple, v) -> tuple[float, float, float]:
    x, y = _rot(base[2], v[0], v[1])
    return (x, y, v[2])


def place_face(place: dict, size, face: str) -> tuple[tuple, tuple]:
    """A desired REST pose is a virtual package: where its plate would be."""
    return magnet_face_pose(place, size, face)


def support_top_z(w: dict | None, x: float, y: float) -> float:
    """Height of whatever a released package at (x, y) lands on: the highest prop
    whose footprint contains the point, else the floor."""
    top = 0.0
    for prop in (w or {}).get("props", []):
        if geometry.contains(world.prop_footprint(prop), (x, y)):
            top = max(top, world.top_z(prop))
    return top


# -- the tick ---------------------------------------------------------------------

class MagnetSim:
    """One magnet and the packages around it, stepped once per physics tick.

    Owns the packages' poses (the Isaac runtime copies `step()`'s moved ids into
    USD), so the capture rule and the snap are testable without a stage."""

    def __init__(self, packages: dict, support: Callable[[float, float], float] | None = None):
        """packages: {id: {"pose": {x, y, z, yaw}, "size": (sx, sy, sz), "face": str}}"""
        self.pk = {pid: {"size": tuple(p["size"]), "face": p["face"],
                         "pose": dict(p["pose"])} for pid, p in packages.items()}
        self.support = support or (lambda x, y: 0.0)
        self.held: str | None = None

    def pose(self, pid: str) -> dict:
        return dict(self.pk[pid]["pose"])

    def poses(self) -> dict:
        return {pid: dict(p["pose"]) for pid, p in self.pk.items()}

    def step(self, tcp, approach, energized: bool) -> dict:
        """Advance one tick. Returns {energized, held, nearest, moved: [ids]}."""
        moved: list[str] = []
        if not energized and self.held is not None:
            moved.append(self.release())
        if energized and self.held is None:
            best = None
            for pid, p in self.pk.items():
                centre, normal = magnet_face_pose(p["pose"], p["size"], p["face"])
                ok, dist, _ = capture(tcp, approach, centre, normal)
                if ok and (best is None or dist < best[1]):
                    best = (pid, dist)
            if best is not None:
                self.held = best[0]
        if self.held is not None:
            p = self.pk[self.held]
            p["pose"] = held_pose(tcp, approach, p["size"], p["face"], p["pose"]["yaw"])
            moved.append(self.held)
        diag = {"energized": bool(energized), "held": self.held,
                "nearest": nearest(tcp, approach,
                                   [(pid, p["pose"], p["size"], p["face"])
                                    for pid, p in self.pk.items() if pid != self.held])}
        diag["moved"] = moved
        return diag

    def release(self) -> str:
        """De-energize: the held package drops straight down onto its support."""
        pid = self.held
        p = self.pk[pid]
        p["pose"]["z"] = self.support(p["pose"]["x"], p["pose"]["y"])
        self.held = None
        return pid
