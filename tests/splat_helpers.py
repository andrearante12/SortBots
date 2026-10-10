"""Synthetic rtabmap-export output for the splat tests (no ROS, no real DB).

Mirrors the layout and formats of a real export, verified 2026-10-10 against
`rtabmap-export --images_id --poses_camera --poses_format 11 --cloud` on
maps/warehouse/map.db: <prefix>_camera_poses.txt, <prefix>_{rgb,depth,calib}/<id>.*,
<prefix>_cloud.ply (PCL binary, vertex then a `camera` element).
"""
from __future__ import annotations

from pathlib import Path

import numpy as np

# base_link -> optical: the quaternion the first row of a real export carries.
OPTICAL_Q = (0.5, -0.5, 0.5, -0.5)

CALIB = """%YAML:1.0
---
camera_name: "{id}"
image_width: 848
image_height: 480
camera_matrix:
   rows: 3
   cols: 3
   data: [ 5.9976543554507452e+02, 0., 424., 0., 5.9976543554507452e+02,
       240., 0., 0., 1. ]
distortion_coefficients:
   rows: 1
   cols: 5
   data: [ 0., 0., 0., 0., 0. ]
distortion_model: plumb_bob
local_transform:
   rows: 3
   cols: 4
   data: [ 0., 0., 1., 0., -1., 0., 0., 0., 0., -1., 0., 1.19 ]
"""


def write_pcl_ply(path: Path, xyz: np.ndarray, rgb: np.ndarray) -> None:
    """A PLY shaped like PCL's: extra float fields, then a `camera` element."""
    n = len(xyz)
    rec = np.zeros(n, dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
                             ("nx", "<f4"), ("ny", "<f4"), ("nz", "<f4"),
                             ("red", "u1"), ("green", "u1"), ("blue", "u1"),
                             ("curvature", "<f4")])
    rec["x"], rec["y"], rec["z"] = xyz.T
    rec["red"], rec["green"], rec["blue"] = rgb.T
    header = (
        "ply\nformat binary_little_endian 1.0\ncomment PCL generated\n"
        f"element vertex {n}\n"
        "property float x\nproperty float y\nproperty float z\n"
        "property float nx\nproperty float ny\nproperty float nz\n"
        "property uchar red\nproperty uchar green\nproperty uchar blue\n"
        "property float curvature\n"
        "element camera 1\nproperty float view_px\nproperty float view_py\n"
        "end_header\n"
    )
    with open(path, "wb") as f:
        f.write(header.encode())
        rec.tofile(f)
        np.zeros(2, "<f4").tofile(f)   # the camera element's payload


def make_export(out_dir: Path, prefix: str, poses: dict[int, tuple[float, float, float]],
                *, missing_rgb: tuple[int, ...] = (), cloud: np.ndarray | None = None) -> None:
    """Write a fake export: one camera per id at `poses[id]`, optical frame."""
    out_dir.mkdir(parents=True, exist_ok=True)
    for sub in ("rgb", "depth", "calib"):
        (out_dir / f"{prefix}_{sub}").mkdir(exist_ok=True)
    lines = ["#timestamp x y z qx qy qz qw id"]
    for i, (node_id, (x, y, z)) in enumerate(sorted(poses.items())):
        qx, qy, qz, qw = OPTICAL_Q
        lines.append(f"{i * 0.5:.6f} {x} {y} {z} {qx} {qy} {qz} {qw} {node_id}")
        if node_id not in missing_rgb:
            (out_dir / f"{prefix}_rgb" / f"{node_id}.jpg").write_bytes(b"\xff\xd8fakejpeg")
        (out_dir / f"{prefix}_depth" / f"{node_id}.png").write_bytes(b"\x89PNGfake")
        (out_dir / f"{prefix}_calib" / f"{node_id}.yaml").write_text(CALIB.format(id=node_id))
    (out_dir / f"{prefix}_camera_poses.txt").write_text("\n".join(lines) + "\n")
    if cloud is None:
        cloud = np.array([[1.0, 0.0, 0.0], [2.0, 0.0, 1.0], [3.0, 1.0, 0.5]], np.float32)
    rgb = np.tile(np.array([[200, 100, 50]], np.uint8), (len(cloud), 1))
    write_pcl_ply(out_dir / f"{prefix}_cloud.ply", cloud.astype(np.float32), rgb)
