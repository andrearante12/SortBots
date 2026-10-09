#!/usr/bin/env python3
"""Score a bench run against the sequence's ground truth. numpy only, no ROS.

    python3 scripts/slam_bench/score.py RUN_DIR --gt SEQ/groundtruth.txt [--poses slam_poses.txt]

Reads RUN_DIR/odom.csv, odom_info.csv and stamps.csv (written by
odom_logger.py and tum_player.py). Estimated poses are base_link; the player
mounts the camera at base_link * T_BASE_CAM, so each pose is moved to the
camera before comparing with TUM's camera ground truth.

ATE: RMS position error after the best rigid alignment (Umeyama, no scale).
Drift: RPE translation per metre travelled, over 1 s windows — the number that
grows when odometry is "prone to drift" even if ATE looks fine on a short run.
"""
from __future__ import annotations

import argparse
import bisect
import csv
import json
import math
import sys
from pathlib import Path

import numpy as np


def quat_to_R(qx, qy, qz, qw):
    n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    x, y, z, w = qx / n, qy / n, qz / n, qw / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def T(R, t):
    m = np.eye(4)
    m[:3, :3], m[:3, 3] = R, t
    return m


def base_to_cam(height: float = 0.4) -> np.ndarray:
    """Same transform tum_player.py publishes: optical axes in base_link."""
    R = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], dtype=float)
    return T(R, [0, 0, height])


def load_gt(path: Path) -> tuple[list[float], list[np.ndarray]]:
    ts, poses = [], []
    for line in path.read_text().splitlines():
        if line and not line.startswith("#"):
            v = [float(x) for x in line.split()]
            ts.append(v[0])
            poses.append(T(quat_to_R(*v[4:8]), v[1:4]))
    return ts, poses


def load_stamp_map(path: Path) -> dict[str, float]:
    with open(path) as f:
        return {r["ros_stamp"]: float(r["dataset_stamp"]) for r in csv.DictReader(f)}


def load_odom(path: Path, stamps: dict[str, float], cam: np.ndarray):
    """(dataset time, camera pose) for every non-lost odom message."""
    out, lost, total = [], 0, 0
    with open(path) as f:
        for r in csv.DictReader(f):
            total += 1
            if int(r["lost"]):
                lost += 1
                continue
            t = stamps.get(r["stamp"])
            if t is None:
                continue
            P = T(quat_to_R(*(float(r[k]) for k in ("qx", "qy", "qz", "qw"))),
                  [float(r[k]) for k in ("x", "y", "z")])
            out.append((t, P @ cam))
    return out, lost, total


def load_tum_poses(path: Path, stamps: dict[str, float] | None, cam: np.ndarray):
    """TUM-format file (t x y z qx qy qz qw) of base_link poses, e.g. the SLAM
    graph exported from the database. Stamps are ROS stamps unless the map
    is None."""
    keys = sorted((float(k), v) for k, v in stamps.items()) if stamps else None
    out = []
    for line in path.read_text().splitlines():
        if not line.strip() or line.startswith("#"):
            continue
        v = [float(x) for x in line.split()]
        t = v[0]
        if keys:
            ks = [k for k, _ in keys]
            i = bisect.bisect_left(ks, t)
            j = min((x for x in (i - 1, i) if 0 <= x < len(ks)), key=lambda x: abs(ks[x] - t))
            if abs(ks[j] - t) > 0.005:
                continue
            t = keys[j][1]
        out.append((t, T(quat_to_R(*v[4:8]), v[1:4]) @ cam))
    return out


def associate(est, gt_t, gt_p, max_dt=0.02):
    pairs = []
    for t, P in est:
        i = bisect.bisect_left(gt_t, t)
        cand = [j for j in (i - 1, i) if 0 <= j < len(gt_t)]
        if not cand:
            continue
        j = min(cand, key=lambda j: abs(gt_t[j] - t))
        if abs(gt_t[j] - t) <= max_dt:
            pairs.append((t, P, gt_p[j]))
    return pairs


def umeyama(src: np.ndarray, dst: np.ndarray) -> np.ndarray:
    """Rigid transform (no scale) minimising |dst - (R src + t)|."""
    ms, md = src.mean(0), dst.mean(0)
    H = (src - ms).T @ (dst - md)
    U, _, Vt = np.linalg.svd(H)
    S = np.eye(3)
    S[2, 2] = np.sign(np.linalg.det(Vt.T @ U.T))
    R = Vt.T @ S @ U.T
    return T(R, md - R @ ms)


def ate(pairs) -> float:
    est = np.array([P[:3, 3] for _, P, _ in pairs])
    gt = np.array([G[:3, 3] for _, _, G in pairs])
    A = umeyama(est, gt)
    aligned = (A[:3, :3] @ est.T).T + A[:3, 3]
    return float(np.sqrt(np.mean(np.sum((aligned - gt) ** 2, axis=1))))


def drift_pct(pairs, window_s: float = 1.0) -> float:
    """Median relative translation error over `window_s`, as % of distance moved."""
    ts = [t for t, _, _ in pairs]
    errs = []
    for i, (t, P, G) in enumerate(pairs):
        j = bisect.bisect_left(ts, t + window_s)
        if j >= len(pairs):
            break
        _, P2, G2 = pairs[j]
        dP = np.linalg.inv(P) @ P2
        dG = np.linalg.inv(G) @ G2
        moved = np.linalg.norm(dG[:3, 3])
        if moved > 0.05:
            errs.append(np.linalg.norm((np.linalg.inv(dG) @ dP)[:3, 3]) / moved)
    return float(100 * np.median(errs)) if errs else float("nan")


def info_stats(path: Path) -> dict:
    if not path.exists():
        return {}
    with open(path) as f:
        rows = list(csv.DictReader(f))
    if len(rows) < 2:
        return {}
    upd = sorted(float(r["update_s"]) for r in rows)
    t = [float(r["stamp"]) for r in rows]
    return {
        "frames_processed": len(rows),
        "odom_hz": round((len(rows) - 1) / (t[-1] - t[0]), 2) if t[-1] > t[0] else None,
        "update_p50_s": round(upd[len(upd) // 2], 3),
        "update_p95_s": round(upd[int(0.95 * (len(upd) - 1))], 3),
        "lost_frames": sum(int(r["lost"]) for r in rows),
        "inliers_p50": sorted(int(r["inliers"]) for r in rows)[len(rows) // 2],
    }


def obstacle_stats(path: Path) -> dict:
    if not path.exists():
        return {}
    with open(path) as f:
        rows = list(csv.DictReader(f))
    if len(rows) < 2:
        return {"obstacle_msgs": len(rows)}
    lat = sorted(float(r["latency_s"]) for r in rows)
    t = [float(r["stamp"]) for r in rows]
    return {
        "obstacle_hz": round((len(rows) - 1) / (t[-1] - t[0]), 2),
        "obstacle_latency_p50_ms": round(1000 * lat[len(lat) // 2], 1),
        "obstacle_latency_p99_ms": round(1000 * lat[int(0.99 * (len(lat) - 1))], 1),
    }


def keyframe_stats(path: Path) -> dict:
    if not path.exists():
        return {}
    with open(path) as f:
        rows = list(csv.DictReader(f))
    if len(rows) < 2:
        return {}
    t = [float(r["stamp"]) for r in rows]
    b = sum(int(r["bytes"]) for r in rows)
    return {"keyframe_hz": round((len(rows) - 1) / (t[-1] - t[0]), 2),
            "keyframe_kB_p50": round(sorted(int(r["bytes"]) for r in rows)[len(rows) // 2] / 1000, 1),
            "link_mbit_s": round(8 * b / (t[-1] - t[0]) / 1e6, 2)}


def score(run: Path, gt_path: Path, poses: Path | None = None, cam_height: float = 0.4) -> dict:
    cam = base_to_cam(cam_height)
    stamps = load_stamp_map(run / "stamps.csv")
    gt_t, gt_p = load_gt(gt_path)
    res = {"frames_published": len(stamps)}
    est, lost, total = load_odom(run / "odom.csv", stamps, cam)
    pairs = associate(est, gt_t, gt_p)
    res.update(info_stats(run / "odom_info.csv"))
    res.update(obstacle_stats(run / "obstacles.csv"))
    res.update(keyframe_stats(run / "keyframes.csv"))
    res["odom_msgs"] = total
    res["odom_lost_msgs"] = lost
    gt_span = (min(gt_t), max(gt_t))
    in_gt = [t for t in stamps.values() if gt_span[0] <= t <= gt_span[1]]
    res["odom_coverage"] = round(len(pairs) / max(1, len(in_gt)), 3)
    if len(pairs) >= 10:
        res["odom_ate_m"] = round(ate(pairs), 3)
        res["odom_drift_pct"] = round(drift_pct(pairs), 1)
    if poses and poses.exists():
        sp = associate(load_tum_poses(poses, stamps, cam), gt_t, gt_p)
        res["slam_nodes_scored"] = len(sp)
        if len(sp) >= 10:
            res["slam_ate_m"] = round(ate(sp), 3)
    return res


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("run", type=Path)
    ap.add_argument("--gt", type=Path, required=True)
    ap.add_argument("--poses", type=Path)
    ap.add_argument("--cam-height", type=float, default=0.4)
    a = ap.parse_args(argv)
    r = score(a.run, a.gt, a.poses, a.cam_height)
    print(json.dumps(r, indent=1))
    (a.run / "score.json").write_text(json.dumps(r, indent=1) + "\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
