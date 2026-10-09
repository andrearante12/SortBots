"""scripts/slam_bench/score.py: the numbers the SLAM offload decision rests on."""
import importlib.util
import math
from pathlib import Path

import numpy as np

spec = importlib.util.spec_from_file_location(
    "score", Path(__file__).resolve().parent.parent / "scripts" / "slam_bench" / "score.py")
S = importlib.util.module_from_spec(spec)
spec.loader.exec_module(S)


def circle(n=300, r=2.0, dt=0.1):
    out = []
    for i in range(n):
        a = i * 0.02
        R = S.quat_to_R(0, 0, math.sin(a / 2), math.cos(a / 2))
        out.append((i * dt, S.T(R, [r * math.cos(a), r * math.sin(a), 0.0])))
    return out


def test_rigid_offset_is_not_error():
    gt = circle()
    off = S.T(S.quat_to_R(0, 0, math.sin(0.4), math.cos(0.4)), [5, -3, 1])
    pairs = [(t, off @ P, P) for t, P in gt]
    assert S.ate(pairs) < 1e-9
    assert S.drift_pct(pairs) < 1e-6


def test_scale_error_shows_as_drift():
    gt = circle()
    pairs = []
    for t, P in gt:
        E = P.copy()
        E[:3, 3] *= 1.1   # odometry over-reads distance by 10%
        pairs.append((t, E, P))
    assert 9.0 < S.drift_pct(pairs) < 11.0
    assert S.ate(pairs) > 0.05


def test_associate_respects_max_dt():
    gt_t = [0.0, 1.0, 2.0]
    gt_p = [np.eye(4)] * 3
    est = [(0.01, np.eye(4)), (1.5, np.eye(4)), (2.019, np.eye(4))]
    assert [t for t, _, _ in S.associate(est, gt_t, gt_p)] == [0.01, 2.019]


def test_base_to_cam_points_optical_z_forward():
    C = S.base_to_cam(0.4)
    # optical +z (forward) must be base +x; optical +x (right) is base -y
    assert np.allclose(C[:3, :3] @ [0, 0, 1], [1, 0, 0])
    assert np.allclose(C[:3, :3] @ [1, 0, 0], [0, -1, 0])
