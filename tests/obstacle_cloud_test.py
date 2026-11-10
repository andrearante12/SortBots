"""nodes/obstacle_cloud.py: depth -> base_link obstacle points, no ROS."""
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "nodes"))
import obstacle_cloud as O  # noqa: E402

K = (600.0, 600.0, 424.0, 240.0)
# Level camera 0.4 m up: optical z forward -> base x, optical x right -> base -y.
R = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], dtype=float)
MOUNT = np.eye(4)
MOUNT[:3, :3], MOUNT[:3, 3] = R, [0.1, 0.0, 0.4]


def test_wall_ahead_lands_at_its_distance_in_base_link():
    d = np.full((480, 848), 2.0, dtype=np.float32)
    pts = O.obstacle_points(d, K, MOUNT)
    assert len(pts) > 1000
    assert np.allclose(pts[:, 0], 2.1, atol=1e-4)          # 2 m + 0.1 m mount offset
    assert pts[:, 2].min() >= 0.05 and pts[:, 2].max() <= 1.5


def test_floor_and_out_of_range_are_dropped():
    h, w = 480, 848
    d = np.full((h, w), np.nan, dtype=np.float32)
    # bottom rows see the floor: depth where the ray hits z = 0 (height 0.4)
    for v in range(300, h):
        d[v, :] = 0.4 * K[1] / (v - K[3])
    d[:50, :] = 9.0                                         # beyond max range
    assert len(O.obstacle_points(d, K, MOUNT)) == 0


def test_tilted_mount_matches_quat_matrix():
    pitch = 0.3  # camera tilted down
    q = (0.0, math.sin(pitch / 2), 0.0, math.cos(pitch / 2))
    T = O.quat_matrix(*q, (0, 0, 0)) @ MOUNT
    d = np.full((480, 848), 1.0, dtype=np.float32)
    p = O.obstacle_points(d, K, T, min_height_m=-10, max_height_m=10)
    # pitched down: the optical axis point (center pixel, 1 m) drops below mount height
    c = O.obstacle_points(d[236:244, 420:428], (600, 600, 4, 4), T,
                          stride=1, min_height_m=-10, max_height_m=10)
    assert c[:, 2].mean() < 0.4 - 0.25 and len(p) > 0


def test_16uc1_millimetres_decode():
    raw = np.array([[1000, 0], [2500, 65535]], dtype=np.uint16)
    m = O.depth_to_metres(raw.tobytes(), "16UC1", 2, 2)
    assert m[0, 0] == 1.0 and m[1, 0] == 2.5
