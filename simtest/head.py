"""Where the head camera is for a given pan and tilt. numpy and the URDF, no Isaac, no ROS.

The camera rides the head, so tilting it moves the picture; but base_link -> camera_optical
was captured once, at pan = tilt = 0, and everything reading depth trusts that transform
(RTAB-Map, the obstacle clouds, the plate detector). A tilted head under that TF would put
every point in the wrong place. scripts/spawn_warehouse.py recomputes it from this as the
joints move.

The chain comes from the URDF; the pose at zero comes from Isaac (`extrinsic`'s `zero`), so
whatever constant separates the two frames cancels out instead of being assumed away.
"""
from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parent.parent
URDF_PATH = REPO_ROOT / "third_party/XLeRobot/simulation/Maniskill/assets/xlerobot/xlerobot.urdf"
BASE_LINK = "base_link"
CAMERA_LINK = "head_camera_rgb_optical_frame"
PAN_JOINT, TILT_JOINT = "head_pan_joint", "head_tilt_joint"


def _rpy(r: float, p: float, y: float) -> np.ndarray:
    return _about((0, 0, 1), y) @ _about((0, 1, 0), p) @ _about((1, 0, 0), r)


def _about(axis, angle: float) -> np.ndarray:
    """Rotation matrix for `angle` about a unit `axis` (Rodrigues)."""
    x, y, z = axis
    k = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]], dtype=float)
    return np.eye(3) + math.sin(angle) * k + (1 - math.cos(angle)) * (k @ k)


def _pose(rot: np.ndarray, t) -> np.ndarray:
    out = np.eye(4)
    out[:3, :3], out[:3, 3] = rot, t
    return out


def quat_to_matrix(q) -> np.ndarray:
    """(x, y, z, w) -> 3x3."""
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def matrix_to_quat(m: np.ndarray) -> tuple[float, float, float, float]:
    """3x3 -> (x, y, z, w), through the largest component so no branch divides by ~0."""
    t = np.trace(m)
    if t > 0:
        s = 2 * math.sqrt(1 + t)
        q = ((m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s, s / 4)
    else:
        i = int(np.argmax(np.diag(m)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = 2 * math.sqrt(1 + m[i, i] - m[j, j] - m[k, k])
        v = [0.0, 0.0, 0.0]
        v[i], v[j], v[k] = s / 4, (m[j, i] + m[i, j]) / s, (m[k, i] + m[i, k]) / s
        q = (*v, (m[k, j] - m[j, k]) / s)
    return tuple(float(c) for c in q)


class HeadKin:
    """The URDF's joints from base_link to the camera's optical frame."""

    def __init__(self, urdf: Path | str = URDF_PATH):
        by_child = {}
        for j in ET.parse(str(urdf)).getroot().findall("joint"):
            o, axis = j.find("origin"), j.find("axis")
            xyz = [float(v) for v in (o.get("xyz", "0 0 0") if o is not None else "0 0 0").split()]
            rpy = [float(v) for v in (o.get("rpy", "0 0 0") if o is not None else "0 0 0").split()]
            by_child[j.find("child").get("link")] = (
                j.get("name"), j.find("parent").get("link"), _pose(_rpy(*rpy), xyz),
                tuple(float(v) for v in axis.get("xyz").split()) if axis is not None else None)
        self.chain, link = [], CAMERA_LINK
        while link != BASE_LINK:
            if link not in by_child:
                raise ValueError(f"no joint chain from {BASE_LINK} to {CAMERA_LINK} (stopped at {link})")
            name, parent, origin, axis = by_child[link]
            self.chain.insert(0, (name, origin, axis))
            link = parent

    def matrix(self, pan: float, tilt: float) -> np.ndarray:
        """base_link <- optical frame, 4x4."""
        angles = {PAN_JOINT: pan, TILT_JOINT: tilt}
        out = np.eye(4)
        for name, origin, axis in self.chain:
            out = out @ origin
            if name in angles:
                out = out @ _pose(_about(axis, angles[name]), (0, 0, 0))
        return out

    def extrinsic(self, pan: float, tilt: float, zero) -> tuple[tuple, tuple]:
        """((tx, ty, tz), (qx, qy, qz, qw)) of the camera at this pan and tilt, given
        `zero`, the same pair as measured with the head at 0 (base_link -> camera_optical)."""
        z = _pose(quat_to_matrix(zero[1]), zero[0])
        m = self.matrix(pan, tilt) @ np.linalg.inv(self.matrix(0.0, 0.0)) @ z
        return tuple(float(v) for v in m[:3, 3]), matrix_to_quat(m[:3, :3])
