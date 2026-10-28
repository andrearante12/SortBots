"""Numpy-only FK/IK for the 5-DOF SO-101, parsed from configs/so101_new_calib.urdf.

No torch / pytorch-kinematics (see nodes/arm_ik.py for why that is a trap): the arm is
four joints in a chain plus a roll, so a damped-least-squares solve on a numeric Jacobian
is plenty and imports in milliseconds. URDF zero == the middle of each joint's calibrated
range, the same convention lerobot's DEGREES mode uses, so joint degrees from the robot map
straight to URDF radians (times a per-joint sign if a motor turns the other way).

Task space is (x, y, z, pitch) in base_link metres/radians over pan, lift, elbow,
wrist_flex. `pitch` is the elevation of the wrist-joint -> tip vector, not a tool-frame
Euler angle, so wrist_roll cannot change it. wrist_roll is left to the caller.
"""
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

URDF = Path(__file__).resolve().parent.parent / "configs" / "so101_new_calib.urdf"
ARM = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
CHAIN = ARM + ["gripper_frame_joint"]  # last one is fixed: wrist_roll frame -> tool tip
PITCH_W = 0.1  # metres per radian: weight of pitch error against position error


def _rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def _rot_z(q):
    c, s = math.cos(q), math.sin(q)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


class SO101Kin:
    def __init__(self, urdf=URDF):
        root = ET.parse(urdf).getroot()
        self.joints = []
        for name in CHAIN:
            j = next(x for x in root.findall("joint") if x.get("name") == name)
            o = j.find("origin")
            T = np.eye(4)
            T[:3, :3] = _rpy(*map(float, o.get("rpy").split()))
            T[:3, 3] = list(map(float, o.get("xyz").split()))
            self.joints.append((T, j.get("type") == "revolute"))
        lim = {x.get("name"): x.find("limit") for x in root.findall("joint") if x.find("limit") is not None}
        self.urdf_limits = {n: (float(lim[n].get("lower")), float(lim[n].get("upper"))) for n in ARM}

    def fk(self, q):
        """q: 5 URDF radians (ARM order) -> (tip xyz, wrist-joint xyz)."""
        T = np.eye(4)
        wrist = None
        for i, (To, rev) in enumerate(self.joints):
            T = T @ To
            if CHAIN[i] == "wrist_flex":
                wrist = T[:3, 3].copy()  # origin of the wrist_flex joint frame
            if rev:
                R = np.eye(4)
                R[:3, :3] = _rot_z(q[i])
                T = T @ R
        return T[:3, 3], wrist

    def pose(self, q):
        """-> (x, y, z, pitch)"""
        tip, wrist = self.fk(q)
        d = tip - wrist
        return np.array([*tip, math.atan2(d[2], math.hypot(d[0], d[1]))])

    def ik(self, target, q0, lo, hi, iters=30, lam=0.02, pitch_w=PITCH_W):
        """Solve pan/lift/elbow/wrist_flex for target (x,y,z,pitch); roll fixed at q0[4].
        Returns (q, ok). `ok` means converged AND inside [lo, hi]."""
        q = np.array(q0, float)
        for _ in range(iters):
            p = self.pose(q)
            e = np.array(target, float) - p
            e[3] = (e[3] + math.pi) % (2 * math.pi) - math.pi
            e[3] *= pitch_w
            if np.linalg.norm(e[:3]) < 5e-4 and abs(e[3]) < max(pitch_w, 1e-3) * 0.01:
                break
            J = np.zeros((4, 4))
            for k in range(4):
                dq = q.copy()
                dq[k] += 1e-4
                d = self.pose(dq) - p
                d[3] = ((d[3] + math.pi) % (2 * math.pi) - math.pi) * pitch_w
                J[:, k] = d / 1e-4
            step = J.T @ np.linalg.solve(J @ J.T + lam**2 * np.eye(4), e)
            q[:4] += np.clip(step, -0.2, 0.2)
        p = self.pose(q)
        err = np.array(target, float) - p
        err[3] = (err[3] + math.pi) % (2 * math.pi) - math.pi
        ok = np.linalg.norm(err[:3]) < 3e-3 and (pitch_w < PITCH_W or abs(err[3]) < math.radians(3)) and bool(
            np.all(q[:4] >= lo[:4]) and np.all(q[:4] <= hi[:4]))
        return q, ok
