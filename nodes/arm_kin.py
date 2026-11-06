#!/usr/bin/env python3
"""Forward and inverse kinematics for the XLeRobot arm, numpy only.

`nodes/arm_ik.py` solves the same chain with pytorch_kinematics, which lives only
in the `lerobot` conda env, and this repo never sources ROS 2 in a conda shell: a
node that imports torch cannot be launched with the stack. The pick controller
needs IK in the stack, so this is the version that runs on system python3: it
parses the URDF's joint origins itself and runs damped least squares on a
finite-difference Jacobian. No torch, no ROS, no rclpy.

THE CHAIN. base_link -> arm_base_joint (fixed) -> Rotation -> Pitch -> Elbow ->
Wrist_Pitch -> Wrist_Roll -> Fixed_Jaw, from third_party/XLeRobot/.../xlerobot.urdf
(the first arm, the one the sim drives). Fixed_Jaw is the link motor 5 turns, so the
magnet effector mounts on it; the TCP is the magnet face, simtest.magnet.TCP_OFFSET_M
along the roll axis. `Jaw` is the stock gripper's joint and is not used (the magnet
replaces the gripper), so a solution always carries Jaw = 0.

WHAT THE ARM CAN DO. Rotation turns the whole arm about the vertical axis; Pitch,
Elbow and Wrist_Pitch have parallel axes, so together they place the TCP in the
vertical plane Rotation selected and tilt the approach in that plane. Wrist_Roll
spins about the approach axis itself, so it changes neither the TCP nor the approach:
it is left at 0. So the arm has 4 useful joints for 4 numbers: the TCP position (3)
plus the approach ELEVATION (1). The approach AZIMUTH is not free, it is radial from
the shoulder, which is why the base has to be put in the right place
(simtest.magnet.dock_pose) and why `solve` reports how far off the requested
approach direction it landed.

All poses here are in base_link. simtest.magnet.to_base/dir_to_base convert from the
map frame.

    python3 nodes/arm_kin.py --fk 0 0.5 1.0 0 0          # TCP + approach for a pose
    python3 nodes/arm_kin.py --ik 0.28 -0.13 0.8         # a horizontal-approach solve
"""
from __future__ import annotations

import argparse
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parent.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from simtest import magnet  # noqa: E402

URDF_PATH = (REPO_ROOT / "third_party" / "XLeRobot" / "simulation" / "Maniskill"
             / "assets" / "xlerobot" / "xlerobot.urdf")

CHAIN = ("arm_base_joint", "Rotation", "Pitch", "Elbow", "Wrist_Pitch", "Wrist_Roll")
ARM_JOINTS = CHAIN[1:]                    # the 5 revolute joints, in order
SOLVED = 4                                # Rotation..Wrist_Pitch; Wrist_Roll stays 0

# IK tuning. Millimetres, because the magnet captures within CAPTURE_M = 30 mm and
# the sim's position drives are good to about this much; so is the finite difference.
POS_TOL_M = 0.004
ELEV_TOL_RAD = 0.03
ELEV_WEIGHT = 0.15            # metres of position error one radian of elevation error is worth
LAMBDA = 0.05                 # damping
STEP_LIMIT = 0.25             # rad, per joint per iteration
MAX_ITERS = 100
# Cheap rejection before any iterating: nothing within the joint limits is further
# from the shoulder than this, or outside this height band (base_link). Measured over
# the joint space (tests/arm_kin_test.py recomputes the band), with slack.
MAX_SHOULDER_DIST_M = 0.55
TCP_Z_BAND_BASE = (0.55, 1.30)
# Starting points for the solver, beyond the caller's own seed: elbow-up poses
# spread over the azimuth range and the pitch-plane's reach.
SEEDS = [(0.0, 0.5, 1.0, 0.0), (0.0, 1.0, 1.5, 0.3), (0.0, 1.5, 1.5, -0.3),
         (0.0, 0.2, 0.6, 0.5), (0.0, 2.2, 2.2, -0.6), (0.0, 1.0, 0.5, 0.0)]
AZIMUTHS = (0.0, 0.6, -0.6, 1.2, -1.2, 1.8, -1.8)


def _rpy(r: float, p: float, y: float) -> np.ndarray:
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p), math.sin(p),
                              math.cos(y), math.sin(y))
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def _axis_rot(axis: np.ndarray, th: float) -> np.ndarray:
    k = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
    return np.eye(3) + math.sin(th) * k + (1 - math.cos(th)) * (k @ k)


class ArmKinError(RuntimeError):
    pass


class ArmKin:
    def __init__(self, urdf: Path | str = URDF_PATH, tcp_offset=magnet.TCP_OFFSET_M):
        try:
            root = ET.parse(str(urdf)).getroot()
        except (OSError, ET.ParseError) as exc:
            raise ArmKinError(f"can't read the arm URDF {urdf}: {exc} "
                              "(git submodule update --init third_party/XLeRobot)") from None
        joints = {j.get("name"): j for j in root.findall("joint")}
        missing = [n for n in CHAIN if n not in joints]
        if missing:
            raise ArmKinError(f"{urdf}: no joint(s) {missing}")
        self._origin, self._axis, self.lower, self.upper = [], [], [], []
        for name in CHAIN:
            j = joints[name]
            o = j.find("origin")
            xyz = [float(v) for v in (o.get("xyz") or "0 0 0").split()]
            rpy = [float(v) for v in (o.get("rpy") or "0 0 0").split()]
            t = np.eye(4)
            t[:3, :3] = _rpy(*rpy)
            t[:3, 3] = xyz
            self._origin.append(t)
            if j.get("type") == "revolute":
                a = np.array([float(v) for v in j.find("axis").get("xyz").split()])
                self._axis.append(a / np.linalg.norm(a))
                lim = j.find("limit")
                self.lower.append(float(lim.get("lower")))
                self.upper.append(float(lim.get("upper")))
            else:
                self._axis.append(None)
        self.lower = np.array(self.lower)
        self.upper = np.array(self.upper)
        self.tcp_offset = np.array(tcp_offset, dtype=float)
        self.approach_local = np.array(magnet.APPROACH_LOCAL, dtype=float)

    # -- forward ----------------------------------------------------------------
    def fk(self, q) -> np.ndarray:
        """base_link <- Fixed_Jaw, 4x4, for the 5 joint angles q[:5]."""
        t = np.eye(4)
        for i, name in enumerate(CHAIN):
            t = t @ self._origin[i]
            if self._axis[i] is not None:
                r = np.eye(4)
                r[:3, :3] = _axis_rot(self._axis[i], float(q[i - 1]))
                t = t @ r
        return t

    def shoulder(self) -> np.ndarray:
        """Where the Rotation joint sits in base_link (the arm's vertical axis)."""
        return (self._origin[0] @ self._origin[1])[:3, 3].copy()

    def tcp(self, q) -> tuple[np.ndarray, np.ndarray]:
        """(TCP position, unit approach direction) in base_link."""
        t = self.fk(q)
        return t[:3, :3] @ self.tcp_offset + t[:3, 3], t[:3, :3] @ self.approach_local

    def clip(self, q) -> np.ndarray:
        return np.minimum(np.maximum(np.asarray(q, dtype=float), self.lower), self.upper)

    # -- inverse ----------------------------------------------------------------
    @staticmethod
    def _elevation(a: np.ndarray) -> float:
        return math.asin(max(-1.0, min(1.0, float(a[2]))))

    def _residual(self, q5, target, elev_t):
        pos, a = self.tcp(q5)
        return np.concatenate([target - pos, [ELEV_WEIGHT * (elev_t - self._elevation(a))]])

    def _iterate(self, q4, target, elev_t):
        q = np.zeros(5)
        q[:SOLVED] = q4
        for _ in range(MAX_ITERS):
            r = self._residual(q, target, elev_t)
            if np.linalg.norm(r[:3]) < POS_TOL_M and abs(r[3]) < ELEV_WEIGHT * ELEV_TOL_RAD:
                return q, True
            jac = np.empty((4, SOLVED))
            for k in range(SOLVED):
                qk = q.copy()
                qk[k] += 1e-5
                jac[:, k] = (r - self._residual(qk, target, elev_t)) / 1e-5
            step = jac.T @ np.linalg.solve(jac @ jac.T + LAMBDA ** 2 * np.eye(4), r)
            big = np.max(np.abs(step))
            if big > STEP_LIMIT:
                step *= STEP_LIMIT / big
            q[:SOLVED] = self.clip(q + np.concatenate([step, [0.0]]))[:SOLVED]
        r = self._residual(q, target, elev_t)
        return q, bool(np.linalg.norm(r[:3]) < POS_TOL_M and abs(r[3]) < ELEV_WEIGHT * ELEV_TOL_RAD)

    def solve(self, target, approach=None, q_seed=None) -> dict | None:
        """Joint angles that put the TCP at `target` (base_link) approaching along
        `approach` (a direction, only its elevation is matched; default horizontal).

        Returns {q: [6 floats, Wrist_Roll=0, Jaw=0], pos_err_m, angle_deg, tcp,
        approach} or None when no seed converges inside the joint limits. angle_deg
        is the full 3-D angle between the achieved approach and the requested one:
        the azimuth is not matched, so the caller decides whether it is small enough
        (the magnet tolerates simtest.magnet.CAPTURE_DEG)."""
        target = np.asarray(target, dtype=float)
        want = np.array([1.0, 0.0, 0.0]) if approach is None else np.asarray(approach, float)
        want = want / np.linalg.norm(want)
        elev_t = self._elevation(want)
        # Aim Rotation at the target first: the arm moves in one vertical plane,
        # so the right azimuth makes the rest a 2-D problem the seeds can find.
        sh = self.shoulder()
        if (np.linalg.norm(target - sh) > MAX_SHOULDER_DIST_M
                or not TCP_Z_BAND_BASE[0] <= target[2] <= TCP_Z_BAND_BASE[1]):
            return None
        az0 = math.atan2(target[1] - sh[1], target[0] - sh[0])
        seeds = []
        if q_seed is not None:
            seeds.append(tuple(float(v) for v in q_seed[:SOLVED]))
        seeds += [(az0,) + s[1:] for s in SEEDS]
        seeds += [(az,) + s[1:] for az in AZIMUTHS for s in SEEDS[:2]]
        for s in seeds:
            q, ok = self._iterate(self.clip(np.array(list(s) + [0.0]))[:SOLVED], target, elev_t)
            if not ok:
                continue
            pos, a = self.tcp(q)
            # The elevation is matched but the azimuth is not, and a steep approach
            # has a second branch pointing the opposite way (the arm folded back over
            # itself). Never return that one to a caller who asked for a heading.
            if math.hypot(want[0], want[1]) > 0.3 and a[0] * want[0] + a[1] * want[1] < 0:
                continue
            ang = math.degrees(math.acos(max(-1.0, min(1.0, float(a @ want)))))
            return {"q": [float(v) for v in q] + [0.0], "pos_err_m": float(np.linalg.norm(target - pos)),
                    "angle_deg": ang, "tcp": pos, "approach": a}
        return None


# Approach elevations plan_reach tries, in order (degrees; negative points the magnet
# downward). Each must still leave the capture rule satisfied once the azimuth error
# is added, so the cap is CAPTURE_DEG minus ANGLE_MARGIN_DEG. A plate low in the
# envelope needs the tilt: at nominal reach a horizontal approach cannot get below
# ~0.70 m (base z), a -20 degree one reaches ~0.67 (docs/manipulation.md has the map).
ELEVATIONS_DEG = (0, -8, 8, -14, 14, -20, 20)
ANGLE_MARGIN_DEG = 8.0


def plan_reach(kin: ArmKin, face_centre, normal, standoff: float = magnet.STANDOFF_M,
               q_seed=None) -> dict | None:
    """Joint solutions for putting the magnet on a plate: a pre-approach pose and the
    contact pose, both in base_link.

    face_centre, normal: the plate's centre and OUTWARD normal in base_link. The
    magnet has to arrive along roughly -normal, but the arm can tilt its approach, and
    a tilt is often what makes a low or close plate reachable at all, so this tries
    ELEVATIONS_DEG in order and takes the first that solves at BOTH poses. The
    pre-approach sits `standoff` back along the (tilted) approach direction, so the
    last move is a straight push onto the plate.

    Returns {pre: solution, final: solution, elevation_deg, angle_deg} or None.
    angle_deg is the achieved approach against the plate's inward normal, the number
    simtest.magnet.capture() will be asked about."""
    n = np.asarray(normal, dtype=float)
    centre = np.asarray(face_centre, dtype=float)
    horiz = math.hypot(n[0], n[1])
    if horiz < 1e-3:                       # a top plate: the magnet comes straight down
        tries = [(-90.0, np.array([0.0, 0.0, -1.0]))]
    else:
        h = np.array([-n[0] / horiz, -n[1] / horiz])
        tries = [(e, np.array([h[0] * math.cos(math.radians(e)), h[1] * math.cos(math.radians(e)),
                               math.sin(math.radians(e))])) for e in ELEVATIONS_DEG]
    limit = magnet.CAPTURE_DEG - ANGLE_MARGIN_DEG
    for elev, a in tries:
        final = kin.solve(centre, a, q_seed)
        if final is None:
            continue
        angle = math.degrees(math.acos(max(-1.0, min(1.0, float(final["approach"] @ -n)))))
        if angle > limit:
            continue
        pre = kin.solve(centre - a * standoff, a, final["q"])
        if pre is None:
            continue
        return {"pre": pre, "final": final, "elevation_deg": elev, "angle_deg": angle}
    return None


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--urdf", type=Path, default=URDF_PATH)
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--fk", nargs=5, type=float, metavar="Q", help="Rotation Pitch Elbow Wrist_Pitch Wrist_Roll")
    g.add_argument("--ik", nargs=3, type=float, metavar="XYZ", help="target in base_link, horizontal approach")
    args = ap.parse_args()
    kin = ArmKin(args.urdf)
    if args.fk:
        pos, a = kin.tcp(args.fk)
        print(f"tcp {np.round(pos, 4).tolist()}  approach {np.round(a, 4).tolist()}")
        return 0
    sol = kin.solve(args.ik)
    if sol is None:
        print("no solution inside the joint limits", file=sys.stderr)
        return 1
    print(f"q {np.round(sol['q'], 4).tolist()}  err {sol['pos_err_m'] * 1000:.1f} mm  "
          f"approach off by {sol['angle_deg']:.1f} deg")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
