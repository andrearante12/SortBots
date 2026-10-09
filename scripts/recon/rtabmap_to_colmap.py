#!/usr/bin/env python3
"""RTAB-Map database export -> COLMAP text model, for Gaussian Splatting.

The workstation half of tasks/slam_offload.md: RTAB-Map (on the workstation)
already holds every keyframe image, its loop-closed pose and a colored cloud.
Gaussian Splatting trainers (gsplat, nerfstudio `splatfacto`, the original
3DGS) take a COLMAP model: posed pinhole images plus seed points. This turns
one into the other, so no second structure-from-motion run is needed.

    # 1. export from the database (bench image or the Jetson image):
    rtabmap-export --images --poses --poses_format 10 --cloud --voxel 0.02 \
        --output_dir EXPORT --output scene ~/.ros/sortbots_robot_0.db
    # 2. convert:
    python3 scripts/recon/rtabmap_to_colmap.py EXPORT/scene --out COLMAP_DIR
    # 3. train, e.g.:  ns-train splatfacto colmap --data COLMAP_DIR

Poses: `--poses_format 10` is base_link in ROS axes (format 1 is NOT: it is
converted to motion-capture/optical axes). Each image's camera pose is
base_link * local_transform (from the per-image calibration RTAB-Map writes),
which is the optical frame — already COLMAP's camera convention (x right,
y down, z forward). COLMAP stores world->camera, hence the inverse.

numpy only, no ROS.
"""
from __future__ import annotations

import argparse
import re
import shutil
import sys
from pathlib import Path

import numpy as np


def quat_to_R(x, y, z, w):
    n = np.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def R_to_quat(R):
    """(w, x, y, z), w >= 0 — COLMAP's order."""
    t = np.trace(R)
    if t > 0:
        s = 0.5 / np.sqrt(t + 1.0)
        q = (0.25 / s, (R[2, 1] - R[1, 2]) * s, (R[0, 2] - R[2, 0]) * s, (R[1, 0] - R[0, 1]) * s)
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        q = ((R[2, 1] - R[1, 2]) / s, 0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s)
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        q = ((R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s)
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        q = ((R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s)
    q = np.array(q)
    return q if q[0] >= 0 else -q


def opencv_yaml_matrix(text: str, name: str) -> np.ndarray:
    """A `name: {rows, cols, data: [...]}` block from RTAB-Map's OpenCV YAML."""
    m = re.search(rf"{name}:\s*\n\s*rows:\s*(\d+)\s*\n\s*cols:\s*(\d+)\s*\n\s*data:\s*\[([^\]]*)\]", text)
    if not m:
        raise ValueError(f"no {name} in calibration")
    rows, cols = int(m.group(1)), int(m.group(2))
    return np.array([float(v) for v in m.group(3).replace("\n", " ").split(",")]).reshape(rows, cols)


def read_calib(path: Path) -> dict:
    text = path.read_text()
    K = opencv_yaml_matrix(text, "camera_matrix")
    L = np.eye(4)
    L[:3, :] = opencv_yaml_matrix(text, "local_transform")
    w = int(re.search(r"image_width:\s*(\d+)", text).group(1))
    h = int(re.search(r"image_height:\s*(\d+)", text).group(1))
    return {"K": K, "local": L, "w": w, "h": h}


def read_poses(path: Path) -> dict[str, np.ndarray]:
    out = {}
    for line in path.read_text().splitlines():
        if not line.strip() or line.startswith("#"):
            continue
        v = line.split()
        P = np.eye(4)
        P[:3, :3] = quat_to_R(*map(float, v[4:8]))
        P[:3, 3] = [float(x) for x in v[1:4]]
        out[v[0]] = P
    return out


_PLY_TYPES = {"float": "f4", "float32": "f4", "double": "f8", "uchar": "u1", "uint8": "u1",
              "char": "i1", "int": "i4", "uint": "u4", "short": "i2", "ushort": "u2"}


def read_ply_vertices(path: Path) -> np.ndarray:
    """Vertex element of a binary little-endian PLY as a numpy structured array."""
    with open(path, "rb") as f:
        header, props, n, in_vertex = [], [], 0, False
        while True:
            line = f.readline().decode("ascii", "replace").strip()
            header.append(line)
            if line.startswith("format") and "binary_little_endian" not in line:
                raise ValueError(f"{path}: only binary_little_endian PLY is supported")
            if line.startswith("element"):
                in_vertex = line.split()[1] == "vertex"
                if in_vertex:
                    n = int(line.split()[2])
            elif line.startswith("property") and in_vertex:
                _, typ, name = line.split()
                props.append((name, "<" + _PLY_TYPES[typ]))
            elif line == "end_header":
                break
        return np.frombuffer(f.read(n * np.dtype(props).itemsize), dtype=props, count=n)


def convert(prefix: Path, out: Path, max_points: int = 100_000, seed: int = 0) -> dict:
    """prefix = EXPORT/scene (rtabmap-export's --output_dir/--output)."""
    poses = read_poses(Path(f"{prefix}_poses.txt"))
    rgb_dir, calib_dir = Path(f"{prefix}_rgb"), Path(f"{prefix}_calib")
    sparse = out / "sparse" / "0"
    images = out / "images"
    sparse.mkdir(parents=True, exist_ok=True)
    images.mkdir(parents=True, exist_ok=True)
    cams, cam_ids, rows = [], {}, []
    for img in sorted(rgb_dir.iterdir()):
        stamp = img.stem
        if stamp not in poses:
            continue
        c = read_calib(calib_dir / f"{stamp}.yaml")
        key = (c["w"], c["h"], *np.round(c["K"][[0, 1, 0, 1], [0, 1, 2, 2]], 6))
        if key not in cam_ids:
            cam_ids[key] = len(cam_ids) + 1
            cams.append(f"{cam_ids[key]} PINHOLE {c['w']} {c['h']} {key[2]} {key[3]} {key[4]} {key[5]}")
        cam_world = poses[stamp] @ c["local"]          # optical frame in the map
        w2c = np.linalg.inv(cam_world)
        q = R_to_quat(w2c[:3, :3])
        t = w2c[:3, 3]
        shutil.copy2(img, images / img.name)
        rows.append(f"{len(rows) + 1} {q[0]:.9f} {q[1]:.9f} {q[2]:.9f} {q[3]:.9f} "
                    f"{t[0]:.6f} {t[1]:.6f} {t[2]:.6f} {cam_ids[key]} {img.name}\n")
    (sparse / "cameras.txt").write_text("# CAMERA_ID MODEL WIDTH HEIGHT fx fy cx cy\n" + "\n".join(cams) + "\n")
    # COLMAP images.txt: two lines per image; the second (2D points) may be empty.
    (sparse / "images.txt").write_text(
        "# IMAGE_ID QW QX QY QZ TX TY TZ CAMERA_ID NAME\n# (no 2D observations)\n"
        + "".join(r + "\n" for r in rows))
    npts = 0
    cloud = Path(f"{prefix}_cloud.ply")
    with open(sparse / "points3D.txt", "w") as f:
        f.write("# POINT3D_ID X Y Z R G B ERROR TRACK[]\n")
        if cloud.exists():
            v = read_ply_vertices(cloud)
            idx = np.arange(len(v))
            if len(v) > max_points:
                idx = np.random.default_rng(seed).choice(len(v), max_points, replace=False)
            has_rgb = all(k in v.dtype.names for k in ("red", "green", "blue"))
            for i, j in enumerate(sorted(idx)):
                p = v[j]
                r, g, b = (p["red"], p["green"], p["blue"]) if has_rgb else (128, 128, 128)
                f.write(f"{i + 1} {p['x']:.5f} {p['y']:.5f} {p['z']:.5f} {r} {g} {b} 0\n")
            npts = len(idx)
    return {"images": len(rows), "cameras": len(cams), "points": npts}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("prefix", type=Path, help="EXPORT/scene, as given to rtabmap-export")
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--max-points", type=int, default=100_000)
    a = ap.parse_args(argv)
    r = convert(a.prefix, a.out, a.max_points)
    print(f"{r['images']} images, {r['cameras']} camera(s), {r['points']} seed points -> {a.out}")
    return 0 if r["images"] else 1


if __name__ == "__main__":
    sys.exit(main())
