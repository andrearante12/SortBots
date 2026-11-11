"""scripts/head_cam.py: the head-camera fit recovers a known head from noisy synthetic samples
and predicts points it was not fitted on, at head angles it never saw."""
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
import head_cam as H  # noqa: E402
from so101_kin import SO101Kin  # noqa: E402

KIN = SO101Kin()
# A plausible head: pan axis 12 cm behind and 30 cm above the shoulder, camera 4 cm ahead of the
# tilt axis looking forward-down; servo signs flipped on pan to prove the sign search works.
TRUE = H.HeadCam([-0.08, 0.02, 0.30, 0.02, -0.01, 0.03, 0.025, 0.01,
                  0.04, 0.0, 0.03, -math.pi / 2 - 0.5, 0.0, -math.pi / 2, 0.050], -1.0, 1.0)


def _samples(rng, head_poses, per_pose, noise_m=0.0015):
    out = []
    for pan, tilt in head_poses:
        T_cam_base = np.linalg.inv(TRUE.matrix(pan, tilt))
        n = 0
        while n < per_pose:
            arm = [rng.uniform(-40, 40), rng.uniform(-80, 10), rng.uniform(-10, 80),
                   rng.uniform(-30, 90), rng.uniform(-90, 90)]
            p_cam = (T_cam_base @ np.append(KIN.tool_point(np.radians(arm), TRUE.magnet_m), 1))[:3]
            if p_cam[2] < 0.15:  # behind the lens or too close: the camera would not see it
                continue
            out.append({"arm_deg": arm, "head_deg": [pan, tilt],
                        "p_cam": list(p_cam + rng.normal(0, noise_m, 3))})
            n += 1
    return out


def test_fit_recovers_the_head_and_generalises():
    rng = np.random.default_rng(1)
    train = _samples(rng, [(0, 0), (20, 0), (-20, 0), (0, 15), (0, -15), (15, 10)], 6)
    hc, err_mm = H.fit(train, KIN)
    assert (hc.pan_sign, hc.tilt_sign) == (-1.0, 1.0)
    assert np.sqrt(np.mean(err_mm ** 2)) < 4.0          # noise is 1.5 mm per axis, ~2.6 mm 3-D
    assert abs(hc.magnet_m - TRUE.magnet_m) < 0.005
    test = _samples(rng, [(10, -8), (-25, 5)], 10, noise_m=0.0)  # head angles never fitted
    err = [np.linalg.norm(hc.to_base(s["p_cam"], *s["head_deg"]) -
                          KIN.tool_point(np.radians(s["arm_deg"]), TRUE.magnet_m)) for s in test]
    assert max(err) < 0.006, max(err)


def test_single_head_pose_is_a_rigid_fit():
    rng = np.random.default_rng(2)
    train = _samples(rng, [(5, -3)], 8)
    hc, err_mm = H.fit(train, KIN, fit_magnet=False, magnet_m=TRUE.magnet_m)
    assert np.sqrt(np.mean(err_mm ** 2)) < 4.0
    s = _samples(rng, [(5, -3)], 1, noise_m=0.0)[0]
    p = hc.to_base(s["p_cam"], *s["head_deg"])
    assert np.linalg.norm(p - KIN.tool_point(np.radians(s["arm_deg"]), TRUE.magnet_m)) < 0.005


def test_kabsch_is_exact_on_clean_points():
    rng = np.random.default_rng(3)
    src = rng.normal(size=(6, 3))
    T = H._T(H._rpy(0.3, -0.2, 1.1), (0.1, -0.2, 0.3))
    dst = (T[:3, :3] @ src.T).T + T[:3, 3]
    assert np.allclose(H.kabsch(src, dst), T)


def test_target_is_measured_from_the_top_edge_midpoint():
    t = H.target_in_marker(0.020)
    assert np.allclose(t, [-0.03568, 0.010 - 0.04793, 0.0])   # left of and below the marker


def test_align_z_takes_the_tilt_from_depth_and_keeps_the_in_plane_rotation():
    R_true = H._rpy(0.0, 0.0, 1.0) @ H._rx(math.radians(180 + 40))   # marker yawed 57 deg
    R_pnp = R_true @ H._rx(math.radians(6))                          # PnP tilt 6 deg off
    R = H.align_z(R_pnp, -R_true[:, 2])                               # normal sign is irrelevant
    assert np.allclose(R[:, 2], R_true[:, 2])
    assert np.allclose(R, R_true, atol=1e-9)


def test_plane_fit_ignores_a_few_off_plane_points():
    rng = np.random.default_rng(4)
    pts = np.c_[rng.uniform(-0.05, 0.05, (200, 2)), np.zeros(200)]
    pts[:10, 2] = 0.05                                                # the box edge behind
    c, n = H.plane_fit(pts)
    assert abs(abs(n[2]) - 1) < 1e-6
