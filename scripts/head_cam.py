"""Head camera -> arm base: the model, its fit from hand-eye samples, and applying it. numpy only.

A point the D435 sees (camera optical frame: x right, y down, z out of the lens) becomes a
point in the SO-101's base_link (x forward along the reach at pan 0, z up, the shoulder_pan
axis at x = 0.0388) through the head's two joints:

    T_base_cam(pan, tilt) = A @ Rz(pan) @ B @ Ry(tilt) @ C

    A  base_link <- pan frame: where the pan axis is (its z) and how it is tilted (6)
    B  pan -> tilt frame: tilt axis forward offset and roll about it (2; offsets along
       either axis commute with that axis' rotation, so A and C carry them)
    C  tilt frame -> camera optical frame (6)

plus the magnet face's distance out along the roll axis (1): 15 numbers. Ruler measurements
only seed them; `fit` solves them from samples of (arm joints, head joints, the 3-D point the
camera measured at a marker taped on the magnet face), so what the ruler, the URDF and
"URDF zero = middle of the calibrated range" got wrong is partly absorbed and the rest shows
up as residual. Which way the head servos turn is not assumed: `fit` tries all four sign pairs
from upright seeds and keeps the best. A flipped sign is also expressible as a half-turn in A or
B, so the signs it reports are a parametrisation, not a statement about the servos.
"""
from __future__ import annotations

import itertools
import json
import math

import numpy as np

PAN_AXIS_X_M = 0.0388   # shoulder_pan axis in base_link (configs/so101_new_calib.urdf)
MAGNET_FACE_M = 0.0471  # flange to magnet face along the roll axis (hardware params.py)


def _rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def _ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def _rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def _T(R, t):
    out = np.eye(4)
    out[:3, :3], out[:3, 3] = R, t
    return out


def _rpy(r, p, y):
    return _rz(y) @ _ry(p) @ _rx(r)


def rpy_of(R):
    """3x3 -> (roll, pitch, yaw), the inverse of _rpy away from pitch = +/-90 deg."""
    return (math.atan2(R[2, 1], R[2, 2]), -math.asin(max(-1.0, min(1.0, R[2, 0]))),
            math.atan2(R[1, 0], R[0, 0]))


class HeadCam:
    def __init__(self, params, pan_sign=1.0, tilt_sign=1.0):
        self.p = np.asarray(params, float)
        self.pan_sign, self.tilt_sign = float(pan_sign), float(tilt_sign)

    @property
    def magnet_m(self):
        return float(self.p[14])

    def matrix(self, pan_deg, tilt_deg):
        """base_link <- camera optical frame, 4x4, at these head angles (degrees, as the bus
        reports them)."""
        p = self.p
        A = _T(_rpy(*p[3:6]), p[0:3])
        B = _T(_rx(p[7]), (p[6], 0.0, 0.0))
        C = _T(_rpy(*p[11:14]), p[8:11])
        Rp = _T(_rz(self.pan_sign * math.radians(pan_deg)), (0, 0, 0))
        Rt = _T(_ry(self.tilt_sign * math.radians(tilt_deg)), (0, 0, 0))
        return A @ Rp @ B @ Rt @ C

    def to_base(self, p_cam, pan_deg, tilt_deg):
        return (self.matrix(pan_deg, tilt_deg) @ np.append(np.asarray(p_cam, float), 1.0))[:3]

    def to_dict(self):
        return {"params": [float(v) for v in self.p], "pan_sign": self.pan_sign,
                "tilt_sign": self.tilt_sign}

    @classmethod
    def from_dict(cls, d):
        return cls(d["params"], d["pan_sign"], d["tilt_sign"])

    def save(self, path, extra=None):
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps({**self.to_dict(), **(extra or {})}, indent=2))

    @classmethod
    def load(cls, path):
        return cls.from_dict(json.loads(path.read_text()))


def kabsch(src, dst):
    """Rigid 4x4 that maps points src (n,3) onto dst (n,3), least squares."""
    src, dst = np.asarray(src, float), np.asarray(dst, float)
    cs, cd = src.mean(0), dst.mean(0)
    U, _, Vt = np.linalg.svd((src - cs).T @ (dst - cd))
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    R = Vt.T @ np.diag([1.0, 1.0, d]) @ U.T
    return _T(R, cd - R @ cs)


def tcp_of(kin, sample, magnet_m, off=None):
    """FK magnet face for a sample; `off` = (lift, elbow, wrist) radians added to its angles."""
    q = np.radians(sample["arm_deg"])
    if off is not None:
        q = q + np.array([0.0, off[0], off[1], off[2], 0.0])
    return kin.tool_point(q, magnet_m)


def residuals(params, samples, kin, signs, fit_magnet=True, magnet_m=MAGNET_FACE_M, touch_z=None):
    """Per-sample base-frame error (camera's point minus FK's magnet face), flattened. params
    past the 15 head-camera numbers, if any, are joint offsets (lift, elbow, wrist; radians)."""
    p = np.array(params, float)
    if not fit_magnet:
        p[14] = magnet_m
    hc = HeadCam(p[:15], *signs)
    off = p[15:18] if len(p) > 15 else None
    out = []
    for s in samples:
        tcp = tcp_of(kin, s, p[14], off)
        out.append(hc.to_base(s["p_cam"], *s["head_deg"]) - tcp)
        if touch_z is not None and s.get("kind") == "touch":
            out.append([tcp[2] - s.get("surface_z", touch_z)])
    return np.concatenate(out)


def _lm(f, x, iters=300, lam=1e-3):
    """Levenberg-Marquardt with a forward-difference Jacobian. 15 parameters: plain numpy is
    plenty, and the lerobot env has no scipy."""
    r = f(x)
    cost = r @ r
    for _ in range(iters):
        J = np.empty((r.size, x.size))
        for k in range(x.size):
            dx = np.zeros_like(x)
            dx[k] = 1e-6
            J[:, k] = (f(x + dx) - r) / 1e-6
        g, H = J.T @ r, J.T @ J
        while True:
            step = np.linalg.solve(H + lam * np.diag(np.diag(H) + 1e-9), -g)
            r_new = f(x + step)
            if r_new @ r_new < cost:
                x, r, cost, lam = x + step, r_new, r_new @ r_new, max(lam / 3, 1e-9)
                break
            lam *= 4
            if lam > 1e8:
                return x, r
        if np.linalg.norm(step) < 1e-10:
            break
    return x, r


def head_groups(samples, bin_deg=2.0):
    groups = {}
    for s in samples:
        groups.setdefault(tuple(round(v / bin_deg) for v in s["head_deg"]), []).append(s)
    return groups


def fit(samples, kin, pan_axis=None, tilt_fwd=0.02, magnet_m=MAGNET_FACE_M, fit_magnet=True,
        fit_offsets=False, touch_z=None):
    """-> (HeadCam, per-sample error in mm[, joint offsets deg]). Needs >= 4 samples at one head
    pose to seed, and several head poses for the pan and tilt axes to be determined at all.

    fit_offsets: also solve shoulder_lift / elbow_flex / wrist_flex offsets (the arm's "middle
    of the swept range" vs the URDF's zero; 11 cm at the magnet on our arm, 2026-11-11). Touching
    a flat table can't see them (lift +d, elbow -2d, wrist +d keeps the face flat and level);
    the camera sees the reach that combination changes. Pan's offset is a rigid turn the camera
    pose absorbs. Returns the offsets as a third value.

    touch_z: the touched surface's height in base_link (m), for touch samples. Touches all on
    one surface leave height loose: the camera's height and the joint offsets trade (a real set
    put the lens 9 cm low and wrist_flex 26 deg out, 2026-11-11). A sample's own "surface_z"
    wins over it."""
    groups = head_groups(samples)
    g0 = max(groups.values(), key=len)
    if len(g0) < 4:
        raise ValueError(f"need >= 4 samples at one head pose to seed the fit, best has {len(g0)}")
    # Seed from the biggest group's rigid fit, treated as if it were taken at its own head
    # angles: put those angles into the seed by undoing them through upright axes.
    T_g = kabsch([s["p_cam"] for s in g0], [tcp_of(kin, s, magnet_m) for s in g0])
    if pan_axis is None:  # a head usually sits behind and below the lens it carries
        pan_axis = T_g[:3, 3] + np.array([-0.03, 0.0, -0.05])
    pan0, tilt0 = g0[0]["head_deg"]
    multi_pose = len(groups) > 1
    best = None
    for signs in (itertools.product((1.0, -1.0), repeat=2) if multi_pose else [(1.0, 1.0)]):
        A = _T(np.eye(3), pan_axis)
        B = _T(np.eye(3), (tilt_fwd, 0.0, 0.0))
        Rp = _T(_rz(signs[0] * math.radians(pan0)), (0, 0, 0))
        Rt = _T(_ry(signs[1] * math.radians(tilt0)), (0, 0, 0))
        C = np.linalg.inv(A @ Rp @ B @ Rt) @ T_g
        x0 = np.array([*pan_axis, 0, 0, 0, tilt_fwd, 0, *C[:3, 3], *rpy_of(C[:3, :3]), magnet_m]
                      + ([0.0, 0.0, 0.0] if fit_offsets else []))
        f = lambda x, sg=signs: residuals(x, samples, kin, sg, fit_magnet, magnet_m, touch_z)
        x, r = _lm(f, x0)
        if best is None or r @ r < best[2] @ best[2]:
            best = (x, signs, r)
    x, signs, r = best
    if not fit_magnet:
        x[14] = magnet_m
    r = residuals(x, samples, kin, signs, fit_magnet, magnet_m)  # the 3-D part only, for the report
    err = np.linalg.norm(r.reshape(-1, 3), axis=1) * 1000.0
    if fit_offsets:
        return HeadCam(x[:15], *signs), err, dict(zip(("shoulder_lift", "elbow_flex", "wrist_flex"),
                                                      np.degrees(x[15:18])))
    return HeadCam(x[:15], *signs), err


# -- the package: where the magnet goes, relative to the marker -----------------------------
# Measured on the packages (2026-11-11): the magnet circle's centre sits 35.68 mm left of and
# 47.93 mm below the MIDPOINT OF THE MARKER'S TOP BLACK EDGE, in the marker's own axes as
# printed (x right, y up; OpenCV's marker frame, corner 0 top-left). Same layout on every package.
TARGET_FROM_TOP_EDGE_M = (-0.03568, -0.04793)


def target_in_marker(marker_m, from_top_edge=TARGET_FROM_TOP_EDGE_M):
    """The magnet target in the marker frame (origin at the marker centre), metres."""
    return np.array([from_top_edge[0], marker_m / 2 + from_top_edge[1], 0.0])


def plane_fit(points, inlier_m=0.005):
    """(centroid, unit normal) of an (n,3) cloud, refit once without points > inlier_m off."""
    pts = np.asarray(points, float)
    for _ in range(2):
        c = pts.mean(0)
        n = np.linalg.svd(pts - c)[2][2]
        keep = np.abs((pts - c) @ n) < inlier_m
        if keep.sum() < 10 or keep.all():
            break
        pts = pts[keep]
    return c, n / np.linalg.norm(n)


def align_z(R, n):
    """R with its z axis turned onto the plane normal n by the smallest rotation: keeps the
    marker's in-plane rotation (which a small marker gets right) and replaces its tilt (which a
    2 cm marker gets wrong by degrees) with the depth plane's."""
    z = R[:, 2]
    n = np.asarray(n, float) * (1.0 if np.dot(n, z) >= 0 else -1.0)
    v, c = np.cross(z, n), float(np.dot(z, n))
    if np.linalg.norm(v) < 1e-9:
        return R.copy()
    k = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return (np.eye(3) + k + k @ k / (1 + c)) @ R
