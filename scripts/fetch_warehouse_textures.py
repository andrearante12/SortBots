#!/usr/bin/env python3
"""Download CC0 PolyHaven textures for the SortBots warehouse scene.

These give RTAB-Map (and any visual-odometry pipeline) enough texture
gradient to track ORB features. The Phase 3 warehouse used untextured
UsdGeom.Cubes which produced 0 inliers per frame — see Phase 4's demo
notes in docs/isaac_sim_phase4.md.

Run after `source scripts/activate_isaac.sh` (or any env with `requests`
or stdlib `urllib`):

    python scripts/fetch_warehouse_textures.py

Writes 5 diffuse 1K jpg textures to `assets/textures/` (~3 MB total,
CC0, committed). Re-running is a no-op when files exist; pass `--force`
to overwrite. Idempotent.
"""
from __future__ import annotations

import argparse
import hashlib
import sys
import urllib.request
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
TEXTURES_DIR = REPO_ROOT / "assets" / "textures"

# Curated 1K diffuse-only PolyHaven assets. All CC0, verified 200 OK.
# Picked for high feature gradient (RTAB-Map's ORB matcher needs edges)
# and for surfaces that match the warehouse roles.
TEXTURES = [
    {
        "asset": "concrete_floor_painted",
        "role": "floor",
        "url": "https://dl.polyhaven.org/file/ph-assets/Textures/jpg/1k/concrete_floor_painted/concrete_floor_painted_diff_1k.jpg",
        "md5": "07cee8ac8966a2f84fd7bc2ecb416de9",
    },
    {
        "asset": "red_brick_03",
        "role": "walls",
        "url": "https://dl.polyhaven.org/file/ph-assets/Textures/jpg/1k/red_brick_03/red_brick_03_diff_1k.jpg",
        # md5 known only after first download; checked for tamper detection
        # in Phase 5+ once we want to pin.
        "md5": None,
    },
    {
        "asset": "rusty_metal_03",
        "role": "shelves",
        "url": "https://dl.polyhaven.org/file/ph-assets/Textures/jpg/1k/rusty_metal_03/rusty_metal_03_diff_1k.jpg",
        "md5": None,
    },
    {
        "asset": "weathered_brown_planks",
        "role": "boxes",
        "url": "https://dl.polyhaven.org/file/ph-assets/Textures/jpg/1k/weathered_brown_planks/weathered_brown_planks_diff_1k.jpg",
        "md5": None,
    },
    {
        "asset": "wood_planks_grey",
        "role": "pallets",
        "url": "https://dl.polyhaven.org/file/ph-assets/Textures/jpg/1k/wood_planks_grey/wood_planks_grey_diff_1k.jpg",
        "md5": None,
    },
]


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Fetch PolyHaven warehouse textures.")
    p.add_argument("--force", action="store_true", help="Overwrite existing files.")
    p.add_argument(
        "--dest",
        default=str(TEXTURES_DIR),
        help=f"Destination directory (default: {TEXTURES_DIR}).",
    )
    return p.parse_args()


def _file_md5(path: Path) -> str:
    h = hashlib.md5()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(8192), b""):
            h.update(chunk)
    return h.hexdigest()


def fetch_one(spec: dict, dest_dir: Path, force: bool) -> Path:
    out = dest_dir / f"{spec['asset']}_diff_1k.jpg"
    if out.exists() and not force:
        if spec["md5"] and _file_md5(out) != spec["md5"]:
            print(f"  WARNING: {out.name} md5 mismatch; re-downloading")
        else:
            print(f"  {spec['asset']:30s} ({spec['role']:8s})  cached")
            return out
    print(f"  {spec['asset']:30s} ({spec['role']:8s})  downloading...", end="", flush=True)
    urllib.request.urlretrieve(spec["url"], out)
    size = out.stat().st_size
    print(f" {size / 1024:.1f} KB")
    if spec["md5"] and _file_md5(out) != spec["md5"]:
        raise RuntimeError(
            f"{out.name}: md5 mismatch after download (expected {spec['md5']})"
        )
    return out


def main() -> int:
    args = parse_args()
    dest = Path(args.dest)
    dest.mkdir(parents=True, exist_ok=True)
    print(f"fetch_warehouse_textures → {dest}")
    for spec in TEXTURES:
        try:
            fetch_one(spec, dest, args.force)
        except Exception as e:
            print(f"  ERROR fetching {spec['asset']}: {e}", file=sys.stderr)
            return 1
    print("done")
    return 0


if __name__ == "__main__":
    sys.exit(main())
