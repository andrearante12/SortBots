"""Pure-numpy fusion of per-robot recon clouds into the world `map` frame.

Each robot's RTAB-Map cloud lives in `{robot_id}/map`. Bringup publishes a
static `map -> {robot_id}/map` anchor (spawn pose). Transforming every
cloud into `map` and concatenating is collaborative *fusion*, not
collaborative SLAM — same contract as nodes/map_fuse.py for 2D grids.

Split from the ROS wrapper so tests/recon_merge_test.py needs no rclpy.
"""
from __future__ import annotations

import math

import numpy as np

MIN_LEAF_GROWTH = 1.1
MAX_LEAF_RETRIES = 8
DEFAULT_VOXEL = 0.05
DEFAULT_MAX_POINTS = 200_000
NO_COLOR = np.array([0xB0, 0xB0, 0xB0, 0], dtype=np.uint8)


def quat_to_rot(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    """3x3 rotation matrix (body -> parent) from quaternion xyzw."""
    x, y, z, w = qx, qy, qz, qw
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def transform_xyz(
    xyz: np.ndarray,
    tx: float,
    ty: float,
    tz: float,
    qx: float = 0.0,
    qy: float = 0.0,
    qz: float = 0.0,
    qw: float = 1.0,
) -> np.ndarray:
    """Apply map <- robot_map transform to Nx3 points."""
    if len(xyz) == 0:
        return xyz.astype(np.float32, copy=False)
    R = quat_to_rot(qx, qy, qz, qw)
    t = np.array([tx, ty, tz], dtype=np.float64)
    out = (xyz.astype(np.float64) @ R.T) + t
    return out.astype(np.float32)


def downsample_xyz(
    xyz: np.ndarray,
    *,
    max_points: int = DEFAULT_MAX_POINTS,
    voxel_size: float = DEFAULT_VOXEL,
) -> tuple[np.ndarray, float]:
    """Return indices to keep and the leaf used (0 if no downsample)."""
    n = len(xyz)
    if n <= max_points:
        return np.arange(n), 0.0

    lo = xyz.min(axis=0)
    leaf = voxel_size
    idx = np.arange(n)
    for _ in range(MAX_LEAF_RETRIES):
        keys = ((xyz - lo) / leaf).astype(np.int64)
        dims = np.maximum(keys.max(axis=0) + 1, 1)
        packed = (keys[:, 0] * dims[1] + keys[:, 1]) * dims[2] + keys[:, 2]
        idx = np.unique(packed, return_index=True)[1]
        if len(idx) <= max_points:
            return np.sort(idx), float(leaf)
        leaf *= max(MIN_LEAF_GROWTH, (len(idx) / max_points) ** 0.5)

    stride = int(math.ceil(len(idx) / max_points))
    return np.sort(idx)[::stride], float(leaf)


def fuse_clouds(
    sources: list[tuple[np.ndarray, np.ndarray | None, tuple[float, ...]]],
    *,
    max_points: int = DEFAULT_MAX_POINTS,
    voxel_size: float = DEFAULT_VOXEL,
) -> tuple[np.ndarray, np.ndarray, float]:
    """Fuse (xyz, rgb_or_None, transform_xyzw) sources into map frame.

    transform is (tx, ty, tz, qx, qy, qz, qw) for map <- robot_map.
    Returns (xyz_fused, rgb_u8 Nx4, leaf).
    """
    if not sources:
        raise ValueError("fuse_clouds needs at least one source")

    parts_xyz = []
    parts_rgb = []
    for xyz, rgb, xf in sources:
        if len(xyz) == 0:
            continue
        tx, ty, tz, qx, qy, qz, qw = xf
        parts_xyz.append(transform_xyz(xyz, tx, ty, tz, qx, qy, qz, qw))
        if rgb is None:
            parts_rgb.append(np.tile(NO_COLOR, (len(xyz), 1)))
        else:
            parts_rgb.append(np.asarray(rgb, dtype=np.uint8).reshape(-1, 4))

    if not parts_xyz:
        return (
            np.zeros((0, 3), dtype=np.float32),
            np.zeros((0, 4), dtype=np.uint8),
            0.0,
        )

    xyz = np.concatenate(parts_xyz, axis=0)
    rgb = np.concatenate(parts_rgb, axis=0)
    idx, leaf = downsample_xyz(xyz, max_points=max_points, voxel_size=voxel_size)
    return xyz[idx], rgb[idx], leaf
