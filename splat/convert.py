"""Gaussian parameters <-> files. Pure numpy, no torch.

Two outputs per model:
  * model.ply   — the standard 3DGS layout (x y z, f_dc_0..2, opacity,
                  scale_0..2, rot_0..3; LOG scales, LOGIT opacity, wxyz quat).
                  Opens in SuperSplat / any 3DGS viewer, and is what a re-train
                  or a future SH-degree bump would start from.
  * model.splat — the compact 32-byte-per-gaussian format the dashboard's
                  vendored renderer (webui/vendor/splat/) reads:
                    pos f32x3 | scale f32x3 (LINEAR) | rgba u8x4 | quat u8x4 (wxyz)
                  Sorted most-visible first, so a truncated download or a
                  vertex budget still shows the important gaussians.

The trainer keeps SH degree 0 on purpose: .splat carries no view-dependent
colour, and degree 3 would quadruple the parameter memory on an 8 GB card for
nothing the dashboard can show.

    python3 splat/convert.py model.ply model.splat [--max-gaussians N]
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from dataset import read_ply_vertices  # noqa: E402

SH_C0 = 0.28209479177387814  # zeroth-order SH basis constant
SPLAT_DTYPE = np.dtype([("pos", "<f4", 3), ("scale", "<f4", 3),
                        ("rgba", "u1", 4), ("rot", "u1", 4)])
assert SPLAT_DTYPE.itemsize == 32

# Pruning. A gaussian with near-zero opacity costs a sort slot and a fragment
# pass and contributes nothing; a gaussian larger than this is almost always a
# "floater" fitted to a moving actor or a sky/void region, and in the browser
# it smears across the whole view.
MIN_OPACITY = 1.0 / 255.0
MAX_SCALE_M = 2.0


def _sigmoid(x):
    return 1.0 / (1.0 + np.exp(-x))


def to_ply_record(means, log_scales, quats_wxyz, sh0, opacity_logit) -> np.ndarray:
    """Raw trainer tensors (as numpy) -> standard 3DGS PLY structured array."""
    n = len(means)
    names = (["x", "y", "z", "nx", "ny", "nz", "f_dc_0", "f_dc_1", "f_dc_2", "opacity"]
             + [f"scale_{i}" for i in range(3)] + [f"rot_{i}" for i in range(4)])
    rec = np.zeros(n, dtype=[(k, "<f4") for k in names])
    rec["x"], rec["y"], rec["z"] = means.T
    sh0 = np.asarray(sh0).reshape(n, 3)
    rec["f_dc_0"], rec["f_dc_1"], rec["f_dc_2"] = sh0.T
    rec["opacity"] = np.asarray(opacity_logit).reshape(n)
    for i in range(3):
        rec[f"scale_{i}"] = log_scales[:, i]
    for i in range(4):
        rec[f"rot_{i}"] = quats_wxyz[:, i]
    return rec


def write_ply(path: Path, rec: np.ndarray) -> None:
    header = ["ply", "format binary_little_endian 1.0", f"element vertex {len(rec)}"]
    header += [f"property float {k}" for k in rec.dtype.names]
    header.append("end_header")
    with open(path, "wb") as f:
        f.write(("\n".join(header) + "\n").encode("ascii"))
        rec.tofile(f)


def ply_to_splat(rec: np.ndarray, *, max_gaussians: int | None = None,
                 min_opacity: float = MIN_OPACITY,
                 max_scale: float = MAX_SCALE_M) -> np.ndarray:
    """3DGS PLY record -> sorted, pruned .splat structured array."""
    pos = np.stack([rec["x"], rec["y"], rec["z"]], axis=1).astype(np.float32)
    scale = np.exp(np.stack([rec[f"scale_{i}"] for i in range(3)], axis=1)).astype(np.float32)
    alpha = _sigmoid(rec["opacity"].astype(np.float64))
    color = 0.5 + SH_C0 * np.stack([rec[f"f_dc_{i}"] for i in range(3)], axis=1)
    rot = np.stack([rec[f"rot_{i}"] for i in range(4)], axis=1).astype(np.float64)

    norm = np.linalg.norm(rot, axis=1)
    keep = (alpha >= min_opacity) & (scale.max(axis=1) <= max_scale) & (norm > 1e-8)
    keep &= np.isfinite(pos).all(axis=1) & np.isfinite(scale).all(axis=1)
    pos, scale, alpha, color, rot, norm = (a[keep] for a in (pos, scale, alpha, color, rot, norm))
    rot = rot / norm[:, None]

    # Most visible first: big, opaque gaussians carry the image.
    order = np.argsort(-(scale.prod(axis=1) * alpha))
    if max_gaussians is not None:
        order = order[:max_gaussians]

    out = np.empty(len(order), dtype=SPLAT_DTYPE)
    out["pos"] = pos[order]
    out["scale"] = scale[order]
    rgba = np.concatenate([color[order], alpha[order, None]], axis=1)
    out["rgba"] = np.clip(rgba * 255.0, 0, 255).round().astype(np.uint8)
    out["rot"] = np.clip(rot[order] * 128.0 + 128.0, 0, 255).round().astype(np.uint8)
    return out


def write_splat(path: Path, splat: np.ndarray) -> None:
    tmp = Path(path).with_suffix(Path(path).suffix + ".tmp")
    splat.tofile(tmp)
    tmp.replace(path)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("ply")
    ap.add_argument("splat")
    ap.add_argument("--max-gaussians", type=int, default=None)
    args = ap.parse_args(argv)
    out = ply_to_splat(read_ply_vertices(Path(args.ply)), max_gaussians=args.max_gaussians)
    write_splat(Path(args.splat), out)
    print(f"{len(out)} gaussians -> {args.splat}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
