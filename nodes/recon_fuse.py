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


# --- free-space-aware fusion (dynamic_seg's map-side half) ---------------------
#
# Concatenating clouds can only ADD: a person robot_0 recorded at 10:00 stays
# in the fused map even after robot_1 has driven through that exact spot and
# seen it empty, which is the cross-robot "mapped twice" ghost. Each robot's
# RTAB-Map OctoMap (recon_cloud_relay --source octomap) carries its own
# ray-traced EMPTY space too, so here one robot's free space may erase another
# robot's occupied voxel.
#
# There is no "newer wins": an OctoMap is a robot's whole history fused, with
# no per-voxel time, so a cloud's stamp says nothing about which robot saw a
# voxel last. The erase rule is instead geometric and conservative, because
# the robots are only aligned by their spawn anchors and a wall a few cm off
# between them must not erase itself:
#   * B's empty set holds the voxel AND at least `min_free_nbrs` of its six
#     face neighbours: interior free space, not a boundary B grazed;
#   * B has nothing occupied within one voxel (the alignment tolerance).
# A robot's own free space never erases its own voxels: RTAB-Map's OctoMap
# already resolved that per robot.
#
# The erase runs on its own, coarser grid (`free_voxel`, 10 cm): empty space is
# a VOLUME, millions of points at the 5 cm display resolution, so the relay
# ships it at 10 cm and every test here (the cell, its neighbours, the
# alignment tolerance) is in those 10 cm cells.
DEFAULT_FREE_VOXEL = 0.10

_KEY_BITS = 21
_KEY_OFF = 1 << (_KEY_BITS - 1)
_NBR6 = np.array([[1, 0, 0], [-1, 0, 0], [0, 1, 0], [0, -1, 0], [0, 0, 1], [0, 0, -1]])
_NBR27 = np.array([[i, j, k] for i in (-1, 0, 1) for j in (-1, 0, 1) for k in (-1, 0, 1)])


def voxel_keys(xyz: np.ndarray, voxel: float) -> np.ndarray:
    """Nx3 points → N int64 keys of their voxel (±1048 km at 1 mm, ample)."""
    ijk = np.floor(np.asarray(xyz, dtype=np.float64) / voxel).astype(np.int64)
    return _pack(ijk)


def _pack(ijk: np.ndarray) -> np.ndarray:
    u = ijk + _KEY_OFF
    return (u[:, 0] << (2 * _KEY_BITS)) | (u[:, 1] << _KEY_BITS) | u[:, 2]


def _unpack(keys: np.ndarray) -> np.ndarray:
    m = (1 << _KEY_BITS) - 1
    return np.stack([(keys >> (2 * _KEY_BITS)) & m, (keys >> _KEY_BITS) & m, keys & m],
                    axis=1) - _KEY_OFF


def _in_sorted(q: np.ndarray, table: np.ndarray) -> np.ndarray:
    """Membership in a SORTED unique table. np.isin re-sorts the table on every
    call, and _count_in makes up to 27 calls per robot pair: 3.8 s per fuse on
    2 x 200k points, against a 2 s publish period. searchsorted is ~50x less."""
    if len(table) == 0:
        return np.zeros(len(q), dtype=bool)
    i = np.searchsorted(table, q)
    i[i == len(table)] = 0
    return table[i] == q


def _count_in(keys: np.ndarray, offsets: np.ndarray, table: np.ndarray) -> np.ndarray:
    """For each key, how many of key+offset are in the sorted `table`."""
    ijk = _unpack(keys)
    n = np.zeros(len(keys), dtype=np.int64)
    for off in offsets:
        n += _in_sorted(_pack(ijk + off), table)
    return n


def free_space_erase(
    occupied: list[np.ndarray],
    empty: list[np.ndarray | None],
    *,
    voxel: float = DEFAULT_VOXEL,
    min_free_nbrs: int = 4,
) -> list[np.ndarray]:
    """Per robot, a bool keep-mask over its occupied points (already in `map`).
    `empty[i]` is robot i's empty-space points in `map`, or None."""
    occ_keys = [voxel_keys(o, voxel) if len(o) else np.zeros(0, np.int64) for o in occupied]
    emp_keys = [np.unique(voxel_keys(e, voxel)) if e is not None and len(e) else None
                for e in empty]
    occ_sets = [np.unique(k) for k in occ_keys]
    keep = [np.ones(len(k), dtype=bool) for k in occ_keys]
    for a, ka in enumerate(occ_keys):
        if len(ka) == 0:
            continue
        uniq, inv = np.unique(ka, return_inverse=True)
        erase = np.zeros(len(uniq), dtype=bool)
        for b, eb in enumerate(emp_keys):
            if b == a or eb is None:
                continue
            cand = ~erase & _in_sorted(uniq, eb)
            if not cand.any():
                continue
            ck = uniq[cand]
            interior = _count_in(ck, _NBR6, eb) >= min_free_nbrs
            near_b_occ = _count_in(ck, _NBR27, occ_sets[b]) > 0
            hit = np.zeros(len(uniq), dtype=bool)
            hit[np.nonzero(cand)[0]] = interior & ~near_b_occ
            erase |= hit
        keep[a] = ~erase[inv]
    return keep


def fuse_clouds_clearing(
    sources: list[tuple[np.ndarray, np.ndarray | None, np.ndarray | None, tuple[float, ...]]],
    *,
    max_points: int = DEFAULT_MAX_POINTS,
    voxel_size: float = DEFAULT_VOXEL,
    free_voxel: float = DEFAULT_FREE_VOXEL,
    min_free_nbrs: int = 4,
) -> tuple[np.ndarray, np.ndarray, float, int]:
    """fuse_clouds with free_space_erase in between. Sources are
    (occupied xyz, rgb or None, empty xyz or None, transform). Returns
    (xyz, rgb, leaf, points erased)."""
    occ, emp, rgbs = [], [], []
    for xyz, rgb, empty, xf in sources:
        occ.append(transform_xyz(xyz, *xf) if len(xyz) else np.zeros((0, 3), np.float32))
        emp.append(transform_xyz(empty, *xf) if empty is not None and len(empty) else None)
        rgbs.append(rgb)
    keep = free_space_erase(occ, emp, voxel=free_voxel, min_free_nbrs=min_free_nbrs)
    erased = int(sum((~k).sum() for k in keep))
    identity = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)
    kept = [(o[k], None if r is None else np.asarray(r, np.uint8).reshape(-1, 4)[k], identity)
            for o, r, k in zip(occ, rgbs, keep, strict=True)]
    if not any(len(o) for o, _, _ in kept):
        return np.zeros((0, 3), np.float32), np.zeros((0, 4), np.uint8), 0.0, erased
    xyz, rgb, leaf = fuse_clouds(kept, max_points=max_points, voxel_size=voxel_size)
    return xyz, rgb, leaf, erased
