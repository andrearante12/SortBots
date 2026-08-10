#!/usr/bin/env python3
"""How much floor has actually been mapped, in square metres.

The dashboard's `explored_pct` (nodes/explorer.py's _publish_status) is a
convenience readout, not a coverage metric: it divides known cells by the
CURRENT occupancy grid's own extent, and that extent grows as RTAB-Map
explores, so the number can fall while the robot is making progress. On the
most complete map this repo has produced it reads 43.8%. This script reports
absolute areas instead, and optionally a percentage against a reference map.

The comparison is a ratio of scalar AREAS, deliberately not a cell-wise
overlay: two RTAB-Map sessions have no common frame (each starts its own map
origin), so "free floor mapped, as a fraction of the best prior map's free
floor" is the strongest claim the data supports. Exceeding 100% means this
run mapped more than the reference did.

Two modes. The file mode is pure numpy + pyyaml, deliberately ROS-free — it
has to work after `scripts/run_demo.sh stop` has torn the stack down:

    scripts/map_coverage.py data/runs/<name>/map/final.yaml
    scripts/map_coverage.py data/runs/<name>/map/final.yaml \\
        --reference data/runs/nvidia_explore_20260801_145700/map/checkpoint_resume002.yaml
    scripts/map_coverage.py data/runs/<name>/map/checkpoint_*.yaml --json

The live mode needs `source /opt/ros/jazzy/setup.bash` and a running stack:

    scripts/map_coverage.py --live --watch 30
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np
import yaml

# map_saver's trinary sentinel pixel values (nav2_map_server's map_io.cpp).
TRINARY_FREE = 254
TRINARY_UNKNOWN = 205
TRINARY_OCCUPIED = 0


def _read_pgm(path: Path) -> np.ndarray:
    """Binary P5 PGM -> (h, w) uint8. Handles comments anywhere in the header."""
    data = path.read_bytes()
    if not data.startswith(b"P5"):
        raise ValueError(f"{path}: not a binary P5 PGM (map_saver --fmt pgm writes P5)")

    # Header: magic, width, height, maxval as whitespace-separated tokens,
    # with '#'-to-end-of-line comments legal between any of them, then EXACTLY
    # one whitespace byte before the raster.
    tokens: list[bytes] = []
    i = 2
    while len(tokens) < 3:
        while i < len(data) and data[i : i + 1].isspace():
            i += 1
        if data[i : i + 1] == b"#":
            while i < len(data) and data[i : i + 1] not in (b"\n", b"\r"):
                i += 1
            continue
        start = i
        while i < len(data) and not data[i : i + 1].isspace():
            i += 1
        tokens.append(data[start:i])
    i += 1  # the single whitespace byte terminating the header

    width, height, maxval = (int(t) for t in tokens)
    if maxval > 255:
        raise ValueError(f"{path}: maxval {maxval} > 255 means 2 bytes/pixel, unsupported")
    expected = width * height
    raster = data[i : i + expected]
    if len(raster) != expected:
        raise ValueError(f"{path}: expected {expected} raster bytes, got {len(raster)}")
    return np.frombuffer(raster, dtype=np.uint8).reshape(height, width)


def classify_file(yaml_path: Path) -> dict:
    """Cell counts + areas for a map_saver-written .yaml/.pgm pair."""
    meta = yaml.safe_load(yaml_path.read_text())
    img_path = Path(meta["image"])
    if not img_path.is_absolute():
        img_path = yaml_path.parent / img_path
    img = _read_pgm(img_path)
    res = float(meta["resolution"])
    mode = meta.get("mode", "trinary")  # map_saver's default when absent
    negate = int(meta.get("negate", 0))

    if mode == "trinary":
        # Classify by SENTINEL, not by threshold. map_saver writes exactly
        # 254/205/0 in trinary mode, and the implied occupancy of the unknown
        # pixel is (255-205)/255 = 0.196078 — while this repo's own saved
        # yamls carry free_thresh: 0.196. That is 8e-5 of margin, and a yaml
        # written with free_thresh: 0.2 (a perfectly ordinary value) would
        # silently reclassify every unknown cell as free and report ~100%
        # coverage on a half-explored map.
        unknown = img == TRINARY_UNKNOWN
        free = img == TRINARY_FREE
        occupied = img == TRINARY_OCCUPIED
        other = ~(unknown | free | occupied)
        if other.any():
            # Scale/raw maps mislabelled as trinary, or a hand-edited PGM.
            occ = (255.0 - img) / 255.0 if negate == 0 else img / 255.0
            occupied |= other & (occ > float(meta["occupied_thresh"]))
            free |= other & (occ < float(meta["free_thresh"]))
            unknown |= other & ~(occupied | free)
    else:
        occ = (255.0 - img) / 255.0 if negate == 0 else img / 255.0
        occupied = occ > float(meta["occupied_thresh"])
        free = occ < float(meta["free_thresh"])
        unknown = ~occupied & ~free

    return _summary(
        source=str(yaml_path),
        res=res,
        shape=img.shape,
        n_free=int(np.count_nonzero(free)),
        n_occupied=int(np.count_nonzero(occupied)),
        n_unknown=int(np.count_nonzero(unknown)),
    )


def classify_grid(grid: np.ndarray, res: float, occupied_thresh: int, source: str) -> dict:
    """Same summary for a live nav_msgs/OccupancyGrid payload."""
    unknown = grid < 0
    occupied = grid >= occupied_thresh
    return _summary(
        source=source,
        res=res,
        shape=grid.shape,
        n_free=int(np.count_nonzero(~unknown & ~occupied)),
        n_occupied=int(np.count_nonzero(occupied)),
        n_unknown=int(np.count_nonzero(unknown)),
    )


def _summary(source, res, shape, n_free, n_occupied, n_unknown) -> dict:
    cell = res * res
    n_known = n_free + n_occupied
    return {
        "source": source,
        "resolution": res,
        "width": int(shape[1]),
        "height": int(shape[0]),
        "cells": {
            "free": n_free,
            "occupied": n_occupied,
            "unknown": n_unknown,
            "known": n_known,
            "total": int(shape[0] * shape[1]),
        },
        "area_m2": {
            "free": round(n_free * cell, 2),
            "occupied": round(n_occupied * cell, 2),
            "unknown": round(n_unknown * cell, 2),
            "known": round(n_known * cell, 2),
            "extent": round(shape[0] * shape[1] * cell, 2),
        },
    }


def _print(s: dict, reference: dict | None) -> None:
    a, c = s["area_m2"], s["cells"]
    print(f"{s['source']}")
    print(
        f"  grid       {s['width']}x{s['height']} @ {s['resolution']:.3f} m "
        f"({a['extent']:.1f} m2 bounding extent)"
    )
    for label in ("free", "occupied", "unknown", "known"):
        print(f"  {label:<10} {c[label]:>9d} cells   {a[label]:>9.1f} m2")
    if reference:
        rf = reference["area_m2"]["free"]
        rk = reference["area_m2"]["known"]
        print(f"  reference  {reference['source']}")
        if rf > 0:
            print(f"  free floor   {100.0 * a['free'] / rf:>6.1f}% of reference ({rf:.1f} m2)")
        if rk > 0:
            print(f"  known area   {100.0 * a['known'] / rk:>6.1f}% of reference ({rk:.1f} m2)")


def _live(args) -> int:
    import rclpy
    from nav_msgs.msg import OccupancyGrid
    from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

    # Must match nodes/explorer.py's MAP_QOS: RTAB-Map publishes /map latched,
    # so a transient-local subscription gets the current map immediately
    # instead of waiting for the next update.
    map_qos = QoSProfile(
        depth=1,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        reliability=QoSReliabilityPolicy.RELIABLE,
    )
    topic = f"/{args.robot_id}/map"

    rclpy.init()
    node = rclpy.create_node("map_coverage")
    latest: list = []
    node.create_subscription(OccupancyGrid, topic, lambda m: latest.append(m), map_qos)
    try:
        while True:
            latest.clear()
            waited = 0.0
            while not latest and waited < 10.0:
                rclpy.spin_once(node, timeout_sec=0.2)
                waited += 0.2
            if not latest:
                print(f"no message on {topic} within 10 s", file=sys.stderr)
                return 1
            msg = latest[-1]
            grid = np.frombuffer(bytes(msg.data), dtype=np.int8).astype(np.int16)
            grid = grid.reshape(msg.info.height, msg.info.width)
            s = classify_grid(grid, msg.info.resolution, args.occupied_thresh, topic)
            if args.json:
                print(json.dumps(s), flush=True)
            else:
                _print(s, _reference(args))
            if not args.watch:
                return 0
            end = node.get_clock().now().nanoseconds + int(args.watch * 1e9)
            while node.get_clock().now().nanoseconds < end:
                rclpy.spin_once(node, timeout_sec=0.2)
            print()
    except KeyboardInterrupt:
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


def _reference(args) -> dict | None:
    return classify_file(Path(args.reference)) if args.reference else None


def main() -> int:
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    p.add_argument("maps", nargs="*", type=Path, help="map_saver .yaml file(s)")
    p.add_argument("--reference", help="a .yaml whose free area is the 100% denominator")
    p.add_argument("--json", action="store_true", help="one JSON object per map")
    p.add_argument("--live", action="store_true", help="read the running stack's /map instead")
    p.add_argument("--robot-id", default="robot_0")
    p.add_argument("--watch", type=float, default=0.0, help="--live: reprint every N seconds")
    p.add_argument("--occupied-thresh", type=int, default=65, help="--live: matches explorer.yaml")
    args = p.parse_args()

    if args.live:
        return _live(args)
    if not args.maps:
        p.error("give one or more map .yaml files, or --live")

    reference = _reference(args)
    for i, m in enumerate(args.maps):
        s = classify_file(m)
        if args.json:
            print(json.dumps(s))
        else:
            if i:
                print()
            _print(s, reference)
    return 0


if __name__ == "__main__":
    sys.exit(main())
