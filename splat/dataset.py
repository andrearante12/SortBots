"""RTAB-Map export -> splat training set. Pure numpy + PyYAML, no torch, no ROS.

A saved map's pose graph (maps/<name>/map.db) already IS a splat dataset: every
keyframe's RGB, depth, intrinsics, and the loop-closure-optimized pose. This
module turns the output of

    rtabmap-export --images_id --poses_camera --poses_format 11 --cloud

(see splat/extract.py) into the "keyframe set" the trainer reads:

    <out>/transforms.json      frames + intrinsics, world = the fleet `map` frame
    <out>/images/<rid>_<id>.jpg
    <out>/depth/<rid>_<id>.png     uint16 millimetres (RTAB-Map's own encoding)
    <out>/points3d.ply             init cloud, xyz + rgb, `map` frame

The keyframe set is the seam the future LIVE mode appends to — nothing here may
assume the frames came from an export.

Conventions (verified against a real export 2026-10-10, maps/warehouse):
  * `--poses_camera` rows are `stamp x y z qx qy qz qw id`, the OPTICAL frame
    (z forward, x right, y down — OpenCV) in `<rid>/map`. The first row of a
    fresh map is q = (0.5, -0.5, 0.5, -0.5): exactly base_link -> optical, at
    the mount height. So these are OpenCV camera-to-world matrices already;
    no axis flip (nerfstudio's OpenGL c2w would need one — we don't use it).
  * `<rid>/map` -> `map` is the static spawn anchor bringup publishes from
    configs/robots.yaml (translation only; see sortbots_bringup.launch.py
    _spawn_pose). Same transform nodes/recon_fuse.py applies to the 3D cloud.
  * Depth PNGs are 16-bit millimetres in sim AND real: RTAB-Map stores sim's
    32FC1 metres as 16UC1 mm internally.
"""
from __future__ import annotations

import json
import os
import re
import shutil
import sys
from pathlib import Path

import numpy as np
import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "nodes"))
from recon_fuse import downsample_xyz, quat_to_rot  # noqa: E402

ROBOTS_CONFIG = REPO_ROOT / "configs" / "robots.yaml"

DEPTH_SCALE = 0.001  # depth PNG units -> metres

# Init-cloud budget. The export is a 1 cm voxel grid (~5 M points for the
# warehouse); gsplat densifies on its own, and starting from millions of
# gaussians blows the 8 GB card before densification even begins.
DEFAULT_INIT_POINTS = 300_000
DEFAULT_INIT_VOXEL = 0.03


class DatasetError(ValueError):
    """The export is missing or malformed."""


# --------------------------------------------------------------------------
# Parsing rtabmap-export output
# --------------------------------------------------------------------------

def parse_camera_poses(text: str) -> dict[int, tuple[float, np.ndarray]]:
    """`--poses_format 11` text -> {node_id: (stamp, 4x4 c2w in <rid>/map)}."""
    poses: dict[int, tuple[float, np.ndarray]] = {}
    for lineno, line in enumerate(text.splitlines(), 1):
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        parts = line.split()
        if len(parts) != 9:
            raise DatasetError(f"poses line {lineno}: expected 9 fields, got {len(parts)}")
        stamp, x, y, z, qx, qy, qz, qw = (float(v) for v in parts[:8])
        T = np.eye(4)
        T[:3, :3] = quat_to_rot(qx, qy, qz, qw)
        T[:3, 3] = (x, y, z)
        poses[int(parts[8])] = (stamp, T)
    return poses


_OPENCV_HEADER = re.compile(r"^%YAML.*$|^---\s*$", re.MULTILINE)


def parse_calib(text: str) -> dict:
    """One `<prefix>_calib/<id>.yaml` (OpenCV FileStorage YAML) -> intrinsics.

    OpenCV writes a `%YAML:1.0` directive PyYAML rejects, and may tag matrices
    `!!opencv-matrix`; strip both rather than pull in cv2 for one file format.
    """
    body = _OPENCV_HEADER.sub("", text).replace("!!opencv-matrix", "")
    raw = yaml.safe_load(body) or {}
    try:
        K = raw["camera_matrix"]["data"]
        return {
            "w": int(raw["image_width"]), "h": int(raw["image_height"]),
            "fl_x": float(K[0]), "fl_y": float(K[4]),
            "cx": float(K[2]), "cy": float(K[5]),
        }
    except (KeyError, IndexError, TypeError) as e:
        raise DatasetError(f"calibration missing field: {e}") from e


_PLY_TYPES = {
    "char": "i1", "int8": "i1", "uchar": "u1", "uint8": "u1",
    "short": "i2", "int16": "i2", "ushort": "u2", "uint16": "u2",
    "int": "i4", "int32": "i4", "uint": "u4", "uint32": "u4",
    "float": "f4", "float32": "f4", "double": "f8", "float64": "f8",
}


def read_ply_vertices(path: Path) -> np.ndarray:
    """Binary little-endian PLY -> structured array of the `vertex` element.

    Just enough PLY for two writers: PCL (rtabmap-export: x y z nx ny nz red
    green blue curvature, then a `camera` element we skip) and our own
    write_points_ply / splat/convert.py. Vertex must be the first element.
    """
    with open(path, "rb") as f:
        if f.readline().strip() != b"ply":
            raise DatasetError(f"{path}: not a PLY file")
        fmt = None
        count = None
        fields: list[tuple[str, str]] = []
        in_vertex = False
        while True:
            line = f.readline()
            if not line:
                raise DatasetError(f"{path}: truncated PLY header")
            tok = line.decode("ascii", "replace").split()
            if not tok:
                continue
            if tok[0] == "format":
                fmt = tok[1]
            elif tok[0] == "element":
                if count is None and tok[1] != "vertex":
                    raise DatasetError(f"{path}: first element is {tok[1]!r}, not vertex")
                in_vertex = tok[1] == "vertex"
                if in_vertex:
                    count = int(tok[2])
            elif tok[0] == "property" and in_vertex:
                if tok[1] == "list":
                    raise DatasetError(f"{path}: list properties on vertex unsupported")
                fields.append((tok[2], "<" + _PLY_TYPES[tok[1]]))
            elif tok[0] == "end_header":
                break
        if fmt != "binary_little_endian":
            raise DatasetError(f"{path}: PLY format {fmt!r} unsupported (need binary_little_endian)")
        if count is None:
            raise DatasetError(f"{path}: no vertex element")
        dtype = np.dtype(fields)
        data = np.fromfile(f, dtype=dtype, count=count)
    if len(data) != count:
        raise DatasetError(f"{path}: expected {count} vertices, read {len(data)}")
    return data


def write_points_ply(path: Path, xyz: np.ndarray, rgb: np.ndarray) -> None:
    """xyz float32 Nx3 + rgb uint8 Nx3 -> binary PLY (trainer init cloud)."""
    n = len(xyz)
    rec = np.empty(n, dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
                             ("red", "u1"), ("green", "u1"), ("blue", "u1")])
    rec["x"], rec["y"], rec["z"] = xyz[:, 0], xyz[:, 1], xyz[:, 2]
    rec["red"], rec["green"], rec["blue"] = rgb[:, 0], rgb[:, 1], rgb[:, 2]
    header = (
        "ply\nformat binary_little_endian 1.0\n"
        f"element vertex {n}\n"
        "property float x\nproperty float y\nproperty float z\n"
        "property uchar red\nproperty uchar green\nproperty uchar blue\n"
        "end_header\n"
    )
    with open(path, "wb") as f:
        f.write(header.encode("ascii"))
        rec.tofile(f)


# --------------------------------------------------------------------------
# Anchors
# --------------------------------------------------------------------------

def spawn_anchor(scene: str, robot_id: str, roster_path: Path = ROBOTS_CONFIG) -> np.ndarray:
    """4x4 `map <- <rid>/map`, exactly what bringup's static anchor publishes."""
    with open(roster_path) as f:
        roster = yaml.safe_load(f)
    for r in roster["robots"]:
        if r["id"] == robot_id:
            spawn = (r.get("spawn") or {}).get(scene)
            if spawn is None:
                raise DatasetError(f"{robot_id} has no spawn.{scene} in {roster_path}")
            T = np.eye(4)
            T[:3, 3] = spawn[:3]
            return T
    raise DatasetError(f"{robot_id!r} not in {roster_path}")


# --------------------------------------------------------------------------
# Building the keyframe set
# --------------------------------------------------------------------------

def _link_or_copy(src: Path, dst: Path) -> None:
    # The export and the dataset live under the same cache dir, so a hardlink
    # is free and lets the worker delete the export without losing frames.
    try:
        os.link(src, dst)
    except OSError:
        shutil.copy2(src, dst)


def export_paths(export_dir: Path, prefix: str) -> dict[str, Path]:
    return {
        "poses": export_dir / f"{prefix}_camera_poses.txt",
        "rgb": export_dir / f"{prefix}_rgb",
        "depth": export_dir / f"{prefix}_depth",
        "calib": export_dir / f"{prefix}_calib",
        "cloud": export_dir / f"{prefix}_cloud.ply",
    }


def load_export(export_dir: Path, prefix: str, robot_id: str, anchor: np.ndarray,
                *, stride: int = 1) -> tuple[list[dict], list[tuple[Path, Path]]]:
    """One robot's export -> (frames in `map`, [(src, dst-relative) files to place]).

    Frames without an image, depth, or calibration file are dropped rather than
    failing the job: the export skips nodes it can't decompress, and one bad
    keyframe shouldn't cost a whole reconstruction.
    """
    p = export_paths(export_dir, prefix)
    if not p["poses"].is_file():
        raise DatasetError(f"no camera poses at {p['poses']}")
    poses = parse_camera_poses(p["poses"].read_text())
    if not poses:
        raise DatasetError(f"{p['poses']}: no poses (empty map?)")

    frames: list[dict] = []
    files: list[tuple[Path, Path]] = []
    calib_cache: dict[str, dict] = {}
    for i, node_id in enumerate(sorted(poses)):
        if stride > 1 and i % stride:
            continue
        rgb = p["rgb"] / f"{node_id}.jpg"
        depth = p["depth"] / f"{node_id}.png"
        calib = p["calib"] / f"{node_id}.yaml"
        if not (rgb.is_file() and calib.is_file()):
            continue
        text = calib.read_text()
        intr = calib_cache.get(text) or calib_cache.setdefault(text, parse_calib(text))
        stamp, c2w = poses[node_id]
        stem = f"{robot_id}_{node_id}"
        frame = {
            "id": node_id,
            "robot_id": robot_id,
            "stamp": stamp,
            "file_path": f"images/{stem}.jpg",
            "transform_matrix": (anchor @ c2w).round(9).tolist(),
            **intr,
        }
        files.append((rgb, Path(frame["file_path"])))
        if depth.is_file():
            frame["depth_file_path"] = f"depth/{stem}.png"
            files.append((depth, Path(frame["depth_file_path"])))
        frames.append(frame)
    return frames, files


def load_cloud(path: Path, anchor: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Export cloud -> (xyz float32 in `map`, rgb uint8 Nx3)."""
    v = read_ply_vertices(path)
    xyz = np.stack([v["x"], v["y"], v["z"]], axis=1).astype(np.float64)
    xyz = (xyz @ anchor[:3, :3].T + anchor[:3, 3]).astype(np.float32)
    names = v.dtype.names
    if all(c in names for c in ("red", "green", "blue")):
        rgb = np.stack([v["red"], v["green"], v["blue"]], axis=1).astype(np.uint8)
    else:
        rgb = np.full((len(v), 3), 128, dtype=np.uint8)
    ok = np.isfinite(xyz).all(axis=1)
    return xyz[ok], rgb[ok]


def build_dataset(sources: list[dict], out_dir: Path, *,
                  init_points: int = DEFAULT_INIT_POINTS,
                  init_voxel: float = DEFAULT_INIT_VOXEL,
                  stride: int = 1, meta: dict | None = None) -> dict:
    """Merge per-robot exports into one keyframe set in the `map` frame.

    `sources`: [{"export_dir", "prefix", "robot_id", "anchor" (4x4)}, ...].
    Returns the transforms.json dict it wrote.
    """
    out_dir = Path(out_dir)
    (out_dir / "images").mkdir(parents=True, exist_ok=True)
    (out_dir / "depth").mkdir(parents=True, exist_ok=True)

    all_frames: list[dict] = []
    clouds_xyz, clouds_rgb = [], []
    for src in sources:
        export_dir = Path(src["export_dir"])
        anchor = np.asarray(src["anchor"], dtype=np.float64)
        frames, files = load_export(export_dir, src["prefix"], src["robot_id"], anchor,
                                    stride=stride)
        for s, rel in files:
            dst = out_dir / rel
            if not dst.exists():
                _link_or_copy(s, dst)
        all_frames.extend(frames)
        cloud = export_paths(export_dir, src["prefix"])["cloud"]
        if cloud.is_file():
            xyz, rgb = load_cloud(cloud, anchor)
            clouds_xyz.append(xyz)
            clouds_rgb.append(rgb)

    if not all_frames:
        raise DatasetError("export produced no usable keyframes")

    if clouds_xyz:
        xyz = np.concatenate(clouds_xyz)
        rgb = np.concatenate(clouds_rgb)
        idx, _ = downsample_xyz(xyz, max_points=init_points, voxel_size=init_voxel)
        xyz, rgb = xyz[idx], rgb[idx]
    else:
        # No cloud (export ran without --cloud): back-project nothing, let the
        # trainer fall back to a random init inside the camera bounds.
        xyz = np.zeros((0, 3), np.float32)
        rgb = np.zeros((0, 3), np.uint8)
    write_points_ply(out_dir / "points3d.ply", xyz, rgb)

    transforms = {
        "frame_convention": "opencv",   # c2w, camera z forward, y down
        "world_frame": "map",
        "depth_unit_scale_factor": DEPTH_SCALE,
        "ply_file_path": "points3d.ply",
        "init_points": int(len(xyz)),
        "meta": meta or {},
        "frames": all_frames,
    }
    tmp = out_dir / ".transforms.json.tmp"
    tmp.write_text(json.dumps(transforms, indent=1))
    os.replace(tmp, out_dir / "transforms.json")
    return transforms
