#!/usr/bin/env python3
"""Train a Gaussian splat from a keyframe set (splat/dataset.py) with gsplat.

Runs in the `splat` conda env (scripts/install_splat.sh), never system python:

    conda run -n splat python splat/train.py DATASET_DIR OUT_DIR [--iters N]

Writes, into OUT_DIR:
  progress.json  {step, iters, loss, psnr, gaussians, elapsed_s, eta_s} — the
                 worker polls this; rewritten atomically every few dozen steps
  model.splat    snapshot every `snapshot_every` steps, then final
  model.ply      final, standard 3DGS layout (splat/convert.py)

Deliberately small next to gsplat's examples/simple_trainer.py: SH degree 0
(the .splat format has no view-dependent colour), one camera per step, the
stock DefaultStrategy for densification, plus an inverse-depth loss against the
keyframe depth. Depth is what makes this work on a warehouse: RTAB-Map's
keyframes are sparse and the walls and floor are nearly textureless, so RGB
alone fits them with floaters.

SIGTERM (worker cancel) stops at the next step and still exports what it has.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import signal
import sys
import time
from pathlib import Path

import numpy as np
import torch
import torch.nn.functional as F
import yaml
from PIL import Image

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from convert import SH_C0, ply_to_splat, to_ply_record, write_ply, write_splat  # noqa: E402
from dataset import read_ply_vertices  # noqa: E402

CONFIG = HERE.parent / "configs" / "splat.yaml"

_stop = False


def _on_term(signum, frame):
    global _stop
    _stop = True


# --------------------------------------------------------------------------
# Data
# --------------------------------------------------------------------------

class Keyframes:
    """All frames preloaded to the GPU as uint8 RGB + float16 metres depth.

    ~1200 keyframes at 424x240 is ~350 MB RGB + ~240 MB depth: cheaper than a
    host->device copy every step, and it leaves most of the 8 GB to gaussians.
    """

    def __init__(self, root: Path, downscale: int, device, depth_min: float, depth_max: float):
        meta = json.loads((root / "transforms.json").read_text())
        self.meta = meta
        frames = meta["frames"]
        scale = float(meta.get("depth_unit_scale_factor", 0.001))
        rgbs, depths, Ks, c2ws = [], [], [], []
        for f in frames:
            img = Image.open(root / f["file_path"]).convert("RGB")
            w, h = img.size[0] // downscale, img.size[1] // downscale
            if downscale > 1:
                img = img.resize((w, h), Image.BILINEAR)
            rgbs.append(torch.from_numpy(np.array(img)))
            if f.get("depth_file_path"):
                d = Image.open(root / f["depth_file_path"])
                d = np.asarray(d, dtype=np.float32) * scale
                # NEAREST, never bilinear: averaging across a depth edge invents
                # a surface halfway between foreground and background.
                d = np.asarray(Image.fromarray(d).resize((w, h), Image.NEAREST))
                d = np.where((d >= depth_min) & (d <= depth_max), d, 0.0)
            else:
                d = np.zeros((h, w), np.float32)
            depths.append(torch.from_numpy(d.astype(np.float16)))
            sx, sy = w / f["w"], h / f["h"]
            Ks.append([[f["fl_x"] * sx, 0, f["cx"] * sx],
                       [0, f["fl_y"] * sy, f["cy"] * sy],
                       [0, 0, 1]])
            c2ws.append(f["transform_matrix"])
        self.w, self.h = w, h
        self.rgb = torch.stack(rgbs).to(device)                   # N,H,W,3 u8
        self.depth = torch.stack(depths).to(device)               # N,H,W  f16
        self.K = torch.tensor(Ks, dtype=torch.float32, device=device)
        c2w = torch.tensor(c2ws, dtype=torch.float32, device=device)
        self.viewmat = torch.linalg.inv(c2w)                      # OpenCV w2c
        self.centers = c2w[:, :3, 3]

    def __len__(self):
        return len(self.rgb)

    def scene_scale(self) -> float:
        # gsplat's convention: 1.1 x the max camera distance from their centroid.
        c = self.centers
        return float((c - c.mean(0)).norm(dim=1).max()) * 1.1


# --------------------------------------------------------------------------
# Model
# --------------------------------------------------------------------------

def knn_mean_dist(x: torch.Tensor, k: int = 3, chunk: int = 1024) -> torch.Tensor:
    """Mean distance to the k nearest neighbours, chunked to bound memory."""
    out = torch.empty(len(x), device=x.device)
    for i in range(0, len(x), chunk):
        d = torch.cdist(x[i:i + chunk], x)
        out[i:i + chunk] = d.topk(k + 1, largest=False).values[:, 1:].mean(1)
    return out


def init_params(root: Path, meta: dict, kf: Keyframes, device, seed: int):
    ply = root / meta.get("ply_file_path", "points3d.ply")
    v = read_ply_vertices(ply) if ply.is_file() else None
    if v is not None and len(v) >= 16:
        xyz = torch.tensor(np.stack([v["x"], v["y"], v["z"]], 1), dtype=torch.float32)
        rgb = torch.tensor(np.stack([v["red"], v["green"], v["blue"]], 1),
                           dtype=torch.float32) / 255.0
    else:
        # No init cloud: random points in the camera bounding box, grey.
        g = torch.Generator().manual_seed(seed)
        lo, hi = kf.centers.min(0).values.cpu() - 2, kf.centers.max(0).values.cpu() + 2
        xyz = lo + (hi - lo) * torch.rand(100_000, 3, generator=g)
        rgb = torch.full((len(xyz), 3), 0.5)
    xyz, rgb = xyz.to(device), rgb.to(device)
    dist = knn_mean_dist(xyz).clamp(min=1e-4)
    n = len(xyz)
    params = torch.nn.ParameterDict({
        "means": torch.nn.Parameter(xyz),
        "scales": torch.nn.Parameter(torch.log(dist)[:, None].repeat(1, 3)),
        "quats": torch.nn.Parameter(torch.tensor([1.0, 0, 0, 0], device=device).repeat(n, 1)),
        "opacities": torch.nn.Parameter(torch.logit(torch.full((n,), 0.1, device=device))),
        "sh0": torch.nn.Parameter(((rgb - 0.5) / SH_C0)[:, None, :]),
    })
    return params


def make_optimizers(params, scene_scale: float):
    # simple_trainer.py's learning rates; means scale with the scene.
    lrs = {"means": 1.6e-4 * scene_scale, "scales": 5e-3, "quats": 1e-3,
           "opacities": 5e-2, "sh0": 2.5e-3}
    return {k: torch.optim.Adam([{"params": params[k], "lr": lr, "name": k}], eps=1e-15)
            for k, lr in lrs.items()}


# --------------------------------------------------------------------------
# Losses
# --------------------------------------------------------------------------

def _gauss_window(size=11, sigma=1.5, device=None):
    x = torch.arange(size, device=device, dtype=torch.float32) - size // 2
    g = torch.exp(-x ** 2 / (2 * sigma ** 2))
    g = g / g.sum()
    return (g[:, None] @ g[None, :])[None, None]


def ssim(a: torch.Tensor, b: torch.Tensor, window: torch.Tensor) -> torch.Tensor:
    """SSIM of two 1,3,H,W images in [0,1] (grouped conv, the reference form)."""
    c = a.shape[1]
    w = window.expand(c, 1, -1, -1)
    pad = window.shape[-1] // 2
    mu_a = F.conv2d(a, w, padding=pad, groups=c)
    mu_b = F.conv2d(b, w, padding=pad, groups=c)
    s_aa = F.conv2d(a * a, w, padding=pad, groups=c) - mu_a ** 2
    s_bb = F.conv2d(b * b, w, padding=pad, groups=c) - mu_b ** 2
    s_ab = F.conv2d(a * b, w, padding=pad, groups=c) - mu_a * mu_b
    c1, c2 = 0.01 ** 2, 0.03 ** 2
    m = ((2 * mu_a * mu_b + c1) * (2 * s_ab + c2)) / ((mu_a ** 2 + mu_b ** 2 + c1) * (s_aa + s_bb + c2))
    return m.mean()


# --------------------------------------------------------------------------
# Export
# --------------------------------------------------------------------------

def export(params, out_dir: Path, *, final: bool) -> int:
    with torch.no_grad():
        rec = to_ply_record(
            params["means"].detach().cpu().numpy(),
            params["scales"].detach().cpu().numpy(),
            F.normalize(params["quats"].detach(), dim=1).cpu().numpy(),
            params["sh0"].detach().cpu().numpy(),
            params["opacities"].detach().cpu().numpy(),
        )
    splat = ply_to_splat(rec)
    write_splat(out_dir / "model.splat", splat)
    if final:
        tmp = out_dir / ".model.ply.tmp"
        write_ply(tmp, rec)
        os.replace(tmp, out_dir / "model.ply")
    return len(splat)


def write_progress(out_dir: Path, data: dict) -> None:
    tmp = out_dir / ".progress.json.tmp"
    tmp.write_text(json.dumps(data))
    os.replace(tmp, out_dir / "progress.json")


# --------------------------------------------------------------------------
# Loop
# --------------------------------------------------------------------------

def train(dataset_dir: Path, out_dir: Path, cfg: dict) -> dict:
    from gsplat import rasterization
    from gsplat.strategy import DefaultStrategy

    device = torch.device("cuda")
    torch.manual_seed(cfg["seed"])
    out_dir.mkdir(parents=True, exist_ok=True)

    kf = Keyframes(dataset_dir, cfg["downscale"], device, cfg["depth_min"], cfg["depth_max"])
    scene_scale = kf.scene_scale()
    params = init_params(dataset_dir, kf.meta, kf, device, cfg["seed"])
    optimizers = make_optimizers(params, scene_scale)
    iters = int(cfg["iters"])
    # Mean position LR decays 100x over the run, as in the 3DGS paper.
    sched = torch.optim.lr_scheduler.ExponentialLR(optimizers["means"], gamma=0.01 ** (1.0 / iters))

    strategy = DefaultStrategy(refine_stop_iter=int(iters * 0.5), verbose=False)
    strategy.check_sanity(params, optimizers)
    state = strategy.initialize_state(scene_scale=scene_scale)
    window = _gauss_window(device=device)
    max_g = int(cfg["max_gaussians"])

    print(f"{len(kf)} frames {kf.w}x{kf.h}, {len(params['means'])} init gaussians, "
          f"scene scale {scene_scale:.1f} m", flush=True)
    t0 = time.monotonic()
    rng = np.random.default_rng(cfg["seed"])
    order = rng.permutation(len(kf))
    loss_ema = psnr_ema = None
    step = 0
    for step in range(iters):
        if _stop:
            print(f"stopping at step {step} (SIGTERM)", flush=True)
            break
        i = int(order[step % len(order)])
        if step % len(order) == len(order) - 1:
            order = rng.permutation(len(kf))
        gt = kf.rgb[i].float()[None] / 255.0                    # 1,H,W,3
        gt_depth = kf.depth[i].float()[None]                     # 1,H,W

        renders, alphas, info = rasterization(
            means=params["means"], quats=params["quats"],
            scales=torch.exp(params["scales"]), opacities=torch.sigmoid(params["opacities"]),
            colors=params["sh0"], sh_degree=0,
            viewmats=kf.viewmat[i:i + 1], Ks=kf.K[i:i + 1], width=kf.w, height=kf.h,
            render_mode="RGB+ED", packed=False,
        )
        rgb = renders[..., :3].clamp(0, 1)
        depth = renders[..., 3]

        strategy.step_pre_backward(params, optimizers, state, step, info)
        l1 = (rgb - gt).abs().mean()
        s = ssim(rgb.permute(0, 3, 1, 2), gt.permute(0, 3, 1, 2), window)
        loss = (1 - cfg["ssim_lambda"]) * l1 + cfg["ssim_lambda"] * (1 - s)
        valid = gt_depth > 0
        if cfg["depth_lambda"] > 0 and valid.any():
            # Inverse depth: weights near geometry (what the camera sees well)
            # over far, noisy D435 returns. Clamped at the sensor's near limit:
            # where nothing is rendered yet "ED" depth is ~0, its inverse is
            # ~1000, and the first smoke run (2026-10-10) had this one term at
            # 20x the RGB loss.
            inv_pred = 1.0 / depth[valid].clamp(min=cfg["depth_min"])
            inv_gt = 1.0 / gt_depth[valid]
            loss = loss + cfg["depth_lambda"] * (inv_pred - inv_gt).abs().mean()
        loss.backward()

        for opt in optimizers.values():
            opt.step()
            opt.zero_grad(set_to_none=True)
        sched.step()
        strategy.step_post_backward(params, optimizers, state, step, info, packed=False)
        # Densification would happily grow past the card; freeze it at the cap.
        if len(params["means"]) >= max_g and strategy.refine_stop_iter > step:
            strategy.refine_stop_iter = step
            print(f"step {step}: {len(params['means'])} gaussians, densification frozen",
                  flush=True)

        with torch.no_grad():
            mse = F.mse_loss(rgb, gt).item()
            psnr = -10 * math.log10(max(mse, 1e-10))
            loss_ema = loss.item() if loss_ema is None else 0.98 * loss_ema + 0.02 * loss.item()
            psnr_ema = psnr if psnr_ema is None else 0.98 * psnr_ema + 0.02 * psnr
        if step % 50 == 0 or step == iters - 1:
            elapsed = time.monotonic() - t0
            write_progress(out_dir, {
                "step": step + 1, "iters": iters, "loss": round(loss_ema, 5),
                "psnr": round(psnr_ema, 2), "gaussians": len(params["means"]),
                "elapsed_s": round(elapsed, 1),
                "eta_s": round(elapsed / (step + 1) * (iters - step - 1), 1),
            })
        if step % 500 == 0:
            print(f"step {step}/{iters} loss {loss_ema:.4f} psnr {psnr_ema:.2f} "
                  f"gaussians {len(params['means'])}", flush=True)
        if cfg["snapshot_every"] and step and step % cfg["snapshot_every"] == 0:
            n = export(params, out_dir, final=False)
            print(f"snapshot at step {step}: {n} gaussians", flush=True)

    n = export(params, out_dir, final=True)
    result = {"step": step + 1, "iters": iters, "gaussians": n,
              "psnr": round(psnr_ema or 0, 2), "elapsed_s": round(time.monotonic() - t0, 1),
              "stopped": _stop}
    write_progress(out_dir, {**result, "loss": round(loss_ema or 0, 5), "eta_s": 0})
    print(f"done: {n} gaussians, psnr {result['psnr']}", flush=True)
    return result


def load_config(path: Path = CONFIG) -> dict:
    with open(path) as f:
        return yaml.safe_load(f)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("dataset", type=Path)
    ap.add_argument("out", type=Path)
    ap.add_argument("--iters", type=int)
    ap.add_argument("--config", type=Path, default=CONFIG)
    args = ap.parse_args(argv)
    cfg = dict(load_config(args.config)["train"])
    if args.iters:
        cfg["iters"] = args.iters
    signal.signal(signal.SIGTERM, _on_term)
    if not torch.cuda.is_available():
        print("no CUDA device visible to torch", file=sys.stderr)
        return 1
    train(args.dataset, args.out, cfg)
    return 0


if __name__ == "__main__":
    sys.exit(main())
