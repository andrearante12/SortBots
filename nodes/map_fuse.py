#!/usr/bin/env python3
"""ROS-free occupancy-grid fusion core for nodes/map_merge.py.

Pure numpy + stdlib math, no rclpy import — same split as nodes/arm_ik.py vs.
the ROS layer that calls it, done here so this module's logic is testable
offline (tests/map_merge_test.py) the way CLAUDE.md's testing section expects
("no ROS"), which importing rclpy at module scope would otherwise rule out.

See nodes/map_merge.py's module docstring for what this fusion is FOR.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class GridInfo:
    """Plain-data mirror of nav_msgs/MapMetaData's spatial fields."""

    resolution: float
    origin_x: float
    origin_y: float
    width: int
    height: int


@dataclass(frozen=True)
class Transform2D:
    """Rigid 2D transform: rotate by yaw_rad about the source frame's origin,
    then translate by (tx, ty), mapping a point in the SOURCE grid's frame
    into the OUTPUT (fused) frame. Built from a `map -> {robot_id}/map`
    TransformStamped; yaw is carried (not just x/y) so a future non-zero spawn
    yaw in configs/robots.yaml works without touching this module.
    """

    tx: float
    ty: float
    yaw_rad: float = 0.0

    def apply(self, x: np.ndarray, y: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        c, s = math.cos(self.yaw_rad), math.sin(self.yaw_rad)
        return c * x - s * y + self.tx, s * x + c * y + self.ty

    def apply_inverse(self, x: np.ndarray, y: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        c, s = math.cos(self.yaw_rad), math.sin(self.yaw_rad)
        lx, ly = x - self.tx, y - self.ty
        return c * lx + s * ly, -s * lx + c * ly


def _world_corners(info: GridInfo, transform: Transform2D) -> tuple[np.ndarray, np.ndarray]:
    """The 4 corners of `info`'s extent, mapped into the output frame."""
    xs = np.array([0.0, info.width * info.resolution])
    ys = np.array([0.0, info.height * info.resolution])
    local_x, local_y = np.meshgrid(info.origin_x + xs, info.origin_y + ys)
    return transform.apply(local_x.ravel(), local_y.ravel())


def _compute_output_info(
    sources: list[tuple[np.ndarray, GridInfo, Transform2D]],
    resolution: float,
    prev_info: GridInfo | None,
) -> GridInfo:
    """Union bounding box of every source's transformed extent, grown (never
    shrunk) against `prev_info` so the fused grid's origin/size don't jitter
    from one publish tick to the next — a shifting origin would show up as
    the costmap and dashboard map view jumping every publish_period_s.
    """
    min_x = math.inf
    min_y = math.inf
    max_x = -math.inf
    max_y = -math.inf
    for _, info, transform in sources:
        cx, cy = _world_corners(info, transform)
        min_x, max_x = min(min_x, cx.min()), max(max_x, cx.max())
        min_y, max_y = min(min_y, cy.min()), max(max_y, cy.max())

    if prev_info is not None:
        min_x = min(min_x, prev_info.origin_x)
        min_y = min(min_y, prev_info.origin_y)
        max_x = max(max_x, prev_info.origin_x + prev_info.width * prev_info.resolution)
        max_y = max(max_y, prev_info.origin_y + prev_info.height * prev_info.resolution)

    width = max(1, math.ceil((max_x - min_x) / resolution))
    height = max(1, math.ceil((max_y - min_y) / resolution))
    return GridInfo(resolution=resolution, origin_x=min_x, origin_y=min_y, width=width, height=height)


def _resample(data: np.ndarray, info: GridInfo, transform: Transform2D, out_info: GridInfo) -> np.ndarray:
    """Nearest-neighbour resample of one source grid into `out_info`'s frame
    and resolution. Cells the source doesn't cover come back -1 (unknown).
    """
    iy, ix = np.meshgrid(np.arange(out_info.height), np.arange(out_info.width), indexing="ij")
    world_x = out_info.origin_x + (ix + 0.5) * out_info.resolution
    world_y = out_info.origin_y + (iy + 0.5) * out_info.resolution
    local_x, local_y = transform.apply_inverse(world_x, world_y)

    src_ix = np.floor((local_x - info.origin_x) / info.resolution).astype(np.int64)
    src_iy = np.floor((local_y - info.origin_y) / info.resolution).astype(np.int64)

    in_bounds = (
        (src_ix >= 0) & (src_ix < info.width) & (src_iy >= 0) & (src_iy < info.height)
    )
    out = np.full((out_info.height, out_info.width), -1, dtype=np.int8)
    out[in_bounds] = data[src_iy[in_bounds], src_ix[in_bounds]]
    return out


def fuse_grids(
    sources: list[tuple[np.ndarray, GridInfo, Transform2D]],
    resolution: float,
    occupied_thresh: int = 50,
    prev_info: GridInfo | None = None,
) -> tuple[np.ndarray, GridInfo]:
    """Fuse `sources` (each a (data, info, transform_to_output_frame) tuple)
    into one grid at `resolution`. Priority where sources disagree about a
    cell: occupied > free > unknown; ties within a tier resolve toward the
    more confident reading (max value among occupied, min value among free).
    """
    if not sources:
        raise ValueError("fuse_grids needs at least one source")

    out_info = _compute_output_info(sources, resolution, prev_info)
    layers = np.stack(
        [_resample(data, info, transform, out_info) for data, info, transform in sources],
        axis=0,
    ).astype(np.int16)

    # tier: 0 unknown, 1 free-ish, 2 occupied-ish.
    tier = np.where(layers < 0, 0, np.where(layers >= occupied_thresh, 2, 1))
    best_tier = tier.max(axis=0)
    # Tie-break score: occupied tier wants the MAX value (most confident
    # obstacle); free tier wants the MIN value (most confident free), done by
    # negating so argmax still picks the winner; unknown tier is never the
    # best_tier unless every source is unknown there, so its score is inert.
    score = np.where(tier == 2, layers, np.where(tier == 1, -layers, -1000))
    score = np.where(tier == best_tier[None, :, :], score, -np.inf)
    winner = np.argmax(score, axis=0)

    fused = np.take_along_axis(layers, winner[None, :, :], axis=0)[0]
    fused = np.where(best_tier == 0, -1, fused).astype(np.int8)
    return fused, out_info


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
