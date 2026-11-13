"""scripts/head_cam.py: the head-camera fit recovers a known head from noisy synthetic samples
and predicts points it was not fitted on, at head angles it never saw."""
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
import head_cam as H  # noqa: E402
from so101_kin import SO101Kin  # noqa: E402

KIN = SO101Kin(offsets=None)  # synthetic: the URDF as is, whatever this machine has measured
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


def test_fit_recovers_arm_joint_offsets_from_touches():
    """The arm's reported angles are off from the URDF's (calibration "middle of the swept
    range"); touches seen by the camera recover those offsets, which a flat table can't."""
    true = np.radians([0, 12.0, -9.0, 5.0, 0])
    k_true = SO101Kin(offsets=true)
    rng = np.random.default_rng(5)

    def touches(pan, tilt, n, heights=(0.0, 0.0, 0.08)):
        T_cb = np.linalg.inv(TRUE.matrix(pan, tilt))
        out = []
        while len(out) < n:
            q = rng.uniform(np.radians([-60, -30, -60, -40, 0]), np.radians([60, 100, 90, 110, 0]))
            F = k_true.flange(q)
            p = (F @ np.array([0, 0, -TRUE.magnet_m, 1]))[:3]
            if F[2, 2] < 0.995 or abs(p[2] - heights[len(out) % len(heights)]) > 0.01:
                continue                       # straight down, on the table or a raised box
            pc = (T_cb @ np.append(p, 1))[:3]
            if pc[2] > 0.15:
                out.append({"arm_deg": list(np.degrees(q)), "head_deg": [pan, tilt],
                            "p_cam": list(pc + rng.normal(0, 0.0015, 3)), "kind": "touch"})
        return out

    samples = (touches(0, 0, 6) + touches(20, 0, 4) + touches(-20, 0, 4)
               + touches(0, 10, 4) + touches(0, -10, 4))
    hc, err, off = H.fit(samples, KIN, magnet_m=TRUE.magnet_m, fit_magnet=False, fit_offsets=True)
    assert abs(off["shoulder_lift"] - 12) < 1 and abs(off["elbow_flex"] + 9) < 1
    assert abs(off["wrist_flex"] - 5) < 1 and np.sqrt(np.mean(err ** 2)) < 4.0


def test_ik_down_reaches_a_magnet_down_pose_and_refuses_out_of_reach():
    rng = np.random.default_rng(8)
    k = SO101Kin(offsets=np.radians([0, -7, -1.5, -13, 0]))     # with calibrated-style offsets
    lo = np.radians([-105, -105, -95, -105, -170.0]); hi = -lo
    seeds, targets = [], []
    while len(targets) < 6:                                      # magnet-down poses: some seeds, some truths
        q = rng.uniform(np.radians([-50, -40, -60, -60, 0]), np.radians([50, 100, 90, 105, 0]))
        F = k.flange(q)
        if F[2, 2] > 0.995:
            (seeds if len(seeds) < 4 else targets).append(q)
    for q_true in targets:
        p = k.tool_point(q_true, H.MAGNET_FACE_M)
        sol = H.ik_down(k, p, seeds, lo, hi)
        assert sol is not None
        q, err, tilt = sol
        assert np.linalg.norm(k.tool_point(q, H.MAGNET_FACE_M) - p) < 0.002 and tilt < 2.0
    assert H.ik_down(k, [0.9, 0.0, 0.0], seeds, lo, hi) is None   # 90 cm out: unreachable


def test_ik_down_pitch_leans_the_flange_axis_toward_the_base():
    k = SO101Kin(offsets=np.radians([0, -7, -1.5, -13, 0]))
    lo = np.radians([-105, -105, -95, -105, -170.0]); hi = -lo
    seed = np.radians([-0.4, 24.1, 31.2, 52.8, 2.2])            # a hand-taught flat pose, 2026-11-13
    p = np.array([0.20, 0.05, 0.02])
    flat = H.ik_down(k, p, [seed], lo, hi)
    trim = H.ik_down(k, p, [seed], lo, hi, pitch_deg=-4.0)
    assert flat is not None and trim is not None
    z = k.flange(trim[0])[:3, 2]
    rad = p[:2] / np.linalg.norm(p[:2])
    assert abs(np.degrees(np.arctan2(z[:2] @ rad, z[2])) + 4.0) < 0.5
    assert np.linalg.norm(k.tool_point(trim[0], H.MAGNET_FACE_M) - p) < 0.002
