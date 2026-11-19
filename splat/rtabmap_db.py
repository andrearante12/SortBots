"""Read an RTAB-Map database directly: `rtabmap-export` without ROS.

    python3 splat/rtabmap_db.py maps/<name>/map.db OUT_DIR --prefix robot_0

writes exactly what splat/dataset.py reads from

    rtabmap-export --images_id --poses_camera --poses_format 11 --cloud

  <prefix>_camera_poses.txt   `stamp x y z qx qy qz qw id`, optical frame, <rid>/map
  <prefix>_rgb/<id>.jpg
  <prefix>_depth/<id>.png     uint16 millimetres
  <prefix>_calib/<id>.yaml    OpenCV FileStorage, the keys dataset.parse_calib reads
  <prefix>_cloud.ply          xyz + rgb, <rid>/map, back-projected from the depth

Why it exists: rtabmap-export is a C++ binary from system Jazzy, and Jazzy has no
packages for every workstation (none for Ubuntu 26.04, the RTX 4070 laptop since
2026-11). numpy + sqlite3 + Pillow is all this needs, so a map can be splatted on any
machine with a GPU. splat/extract.py still prefers rtabmap-export where it exists.

What the DB holds, and what it doesn't (checked 2026-11-17 on a 0.22.1 DB):
  * Node.pose is the ODOMETRY pose of base_link, not the optimized one. Admin.opt_poses
    is only written by some save paths and was NULL in the library map, so the
    loop-closure-corrected poses are recomputed here: a Gauss-Newton pose graph over
    Link (optimize_graph). RTAB-Map's own optimizer (GTSAM/g2o) solves the same
    least-squares problem on the same constraints; Optimizer/Robust is off by default,
    so no robust kernel here either.
  * Links with an all-zero transform are RTAB-Map's null Transform (a "bad
    signature" node kept in the graph, weight -9). They constrain nothing and are
    skipped; a node only they connect falls out of the graph, as it does in the export.
  * Depth is either 16-bit PNG millimetres or, with Mem/SaveDepth16Format false (the
    default), float32 metres bit-cast into an 8-bit 4-channel PNG. cv::imencode
    writes a 4-channel Mat as BGRA, so the float's bytes come back in B,G,R,A order:
    the other order gives depth that looks plausible pixel by pixel (the exponent
    byte doesn't move) but jumps ~0.4 m between neighbours.
  * calibration is an int32[11] header (version x3, type, width, height, then the
    sizes of K, D, R, P and the local transform) followed by K D R P as float64 and
    the base_link -> optical local transform as float32[12].
"""
from __future__ import annotations

import argparse
import io
import json
import math
import sqlite3
import struct
import sys
from collections import defaultdict, deque
from pathlib import Path

import numpy as np

# Link.type values (rtabmap/core/Link.h).
NEIGHBOR, GLOBAL_CLOSURE, LOCAL_SPACE, LOCAL_TIME, USER_CLOSURE, VIRTUAL, \
    NEIGHBOR_MERGED, POSE_PRIOR, LANDMARK, GRAVITY = range(10)
# The relative constraints the optimizer uses. Gravity (a roll/pitch prior from the
# IMU or odometry) and landmarks are left out: a ground robot's roll/pitch barely
# moves, and no SortBots map uses landmarks.
RELATIVE_LINKS = {NEIGHBOR, GLOBAL_CLOSURE, LOCAL_SPACE, LOCAL_TIME, USER_CLOSURE,
                  VIRTUAL, NEIGHBOR_MERGED}

# Back-projection for the init cloud: every Nth pixel, inside the range the D435 is
# trustworthy. dataset.build_dataset voxels it again to its own budget (3 cm).
CLOUD_DECIMATION = 4
CLOUD_DEPTH_MIN, CLOUD_DEPTH_MAX = 0.3, 6.0
CLOUD_VOXEL = 0.02
CLOUD_CHUNK_POINTS = 4_000_000


class DbError(ValueError):
    """The database is missing something an export needs."""


# --------------------------------------------------------------------------
# SE(3) helpers, batched over the leading axis
# --------------------------------------------------------------------------

def so3_exp(w: np.ndarray) -> np.ndarray:
    """(N,3) rotation vectors -> (N,3,3)."""
    w = np.atleast_2d(w)
    th = np.linalg.norm(w, axis=1)
    small = th < 1e-9
    k = np.where(small[:, None], 0.0, w / np.where(small, 1.0, th)[:, None])
    K = np.zeros((len(w), 3, 3))
    K[:, 0, 1], K[:, 0, 2], K[:, 1, 2] = -k[:, 2], k[:, 1], -k[:, 0]
    K[:, 1, 0], K[:, 2, 0], K[:, 2, 1] = k[:, 2], -k[:, 1], k[:, 0]
    s, c = np.sin(th)[:, None, None], np.cos(th)[:, None, None]
    R = np.eye(3) + s * K + (1 - c) * (K @ K)
    # First order where the angle is ~0: exact enough and never divides by zero.
    Wx = np.zeros((len(w), 3, 3))
    Wx[:, 0, 1], Wx[:, 0, 2], Wx[:, 1, 2] = -w[:, 2], w[:, 1], -w[:, 0]
    Wx[:, 1, 0], Wx[:, 2, 0], Wx[:, 2, 1] = w[:, 2], -w[:, 1], w[:, 0]
    return np.where(small[:, None, None], np.eye(3) + Wx, R)


def so3_log(R: np.ndarray) -> np.ndarray:
    """(N,3,3) -> (N,3) rotation vectors."""
    R = np.asarray(R).reshape(-1, 3, 3)
    tr = np.clip((np.trace(R, axis1=1, axis2=2) - 1) / 2, -1.0, 1.0)
    th = np.arccos(tr)
    v = np.stack([R[:, 2, 1] - R[:, 1, 2], R[:, 0, 2] - R[:, 2, 0],
                  R[:, 1, 0] - R[:, 0, 1]], axis=1)
    s = np.sin(th)
    scale = np.where(s > 1e-7, th / (2 * np.where(s > 1e-7, s, 1.0)), 0.5)
    out = v * scale[:, None]
    # Near pi the antisymmetric part vanishes; take the axis from R + I instead.
    near_pi = th > math.pi - 1e-4
    for i in np.flatnonzero(near_pi):
        B = (R[i] + np.eye(3)) / 2
        axis = B[np.argmax(np.diag(B))]
        out[i] = axis / np.linalg.norm(axis) * th[i]
    return out


def rot_to_quat(R: np.ndarray) -> tuple[float, float, float, float]:
    """3x3 -> (qx, qy, qz, qw), qw >= 0."""
    m = R
    t = m[0, 0] + m[1, 1] + m[2, 2]
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        q = ((m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s, s / 4)
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        q = (s / 4, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s, (m[2, 1] - m[1, 2]) / s)
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        q = ((m[0, 1] + m[1, 0]) / s, s / 4, (m[1, 2] + m[2, 1]) / s, (m[0, 2] - m[2, 0]) / s)
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
        q = ((m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, s / 4, (m[1, 0] - m[0, 1]) / s)
    qx, qy, qz, qw = q
    n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    sign = -1.0 if qw < 0 else 1.0
    return (sign * qx / n, sign * qy / n, sign * qz / n, sign * qw / n)


def mat34(blob: bytes | None) -> np.ndarray | None:
    """RTAB-Map's 3x4 float32 transform -> 4x4, or None for a null transform."""
    if not blob or len(blob) < 48:
        return None
    v = np.frombuffer(blob[:48], "<f4").astype(np.float64)
    if not np.any(v):
        return None
    T = np.eye(4)
    T[:3, :] = v.reshape(3, 4)
    # Stored as float32: re-orthonormalise so composing hundreds of them doesn't drift.
    U, _, Vt = np.linalg.svd(T[:3, :3])
    T[:3, :3] = U @ Vt
    return T


# --------------------------------------------------------------------------
# Reading
# --------------------------------------------------------------------------

def connect(db: Path) -> sqlite3.Connection:
    """Read-only, so a library map (a git-lfs object) can never be dirtied."""
    db = Path(db)
    if not db.is_file():
        raise DbError(f"no database at {db}")
    return sqlite3.connect(f"file:{db.resolve()}?mode=ro", uri=True)


def read_nodes(con) -> dict[int, dict]:
    nodes = {}
    for nid, map_id, weight, stamp, pose in con.execute(
            "SELECT id, map_id, weight, stamp, pose FROM Node"):
        T = mat34(pose)
        if T is not None:
            nodes[nid] = {"map_id": map_id, "weight": weight, "stamp": stamp or 0.0, "pose": T}
    return nodes


def read_links(con) -> list[dict]:
    """Relative constraints, one per (unordered pair, type); null transforms skipped.

    Kept in the direction stored, with RTAB-Map's convention pose_to = pose_from @ T
    (checked 2026-11-17: neighbour link 3 -> 1 holds Rz(-53 deg) where node 3's odometry
    is Rz(+53 deg) and node 1's the identity). Flipping a link would also need its
    information matrix moved through the adjoint, so nothing here flips one.
    """
    seen = set()
    links = []
    for f, t, typ, info, tr in con.execute(
            "SELECT from_id, to_id, type, information_matrix, transform FROM Link"):
        if typ not in RELATIVE_LINKS or f == t or f < 0 or t < 0:
            continue
        T = mat34(tr)
        if T is None:
            continue
        key = (min(f, t), max(f, t), typ)
        if key in seen:
            # The DB can hold both directions of one link.
            continue
        seen.add(key)
        I = np.frombuffer(info, "<f8").reshape(6, 6) if info and len(info) == 288 else np.eye(6)
        links.append({"from": f, "to": t, "type": typ, "T": T, "info": I})
    return links


def parse_calibration(blob: bytes) -> dict:
    """Data.calibration -> {fx, fy, cx, cy, width, height, local (4x4 base_link->optical)}."""
    if not blob:
        raise DbError("node has no calibration")
    if len(blob) >= 44:
        hdr = struct.unpack("<11i", blob[:44])
        major, minor = hdr[0], hdr[1]
        ksz, dsz, rsz, psz, ltsz = hdr[6:11]
        if major == 0 and 18 <= minor < 100 and ksz == 9 and ltsz == 12 \
                and len(blob) >= 44 + 8 * (ksz + dsz + rsz + psz) + 48:
            off = 44
            K = struct.unpack_from(f"<{ksz}d", blob, off)
            off += 8 * (ksz + dsz + rsz + psz)
            local = np.eye(4)
            local[:3, :] = np.frombuffer(blob[off:off + 48], "<f4").reshape(3, 4)
            return {"fx": K[0], "fy": K[4], "cx": K[2], "cy": K[5],
                    "width": hdr[4], "height": hdr[5], "local": local, "type": hdr[3]}
    # Pre-0.18 DBs: fx fy cx cy, then the local transform, all float32; no image size.
    if len(blob) == 64:
        v = struct.unpack("<16f", blob)
        local = np.eye(4)
        local[:3, :] = np.array(v[4:16]).reshape(3, 4)
        return {"fx": v[0], "fy": v[1], "cx": v[2], "cy": v[3],
                "width": 0, "height": 0, "local": local, "type": 0}
    raise DbError(f"unrecognised calibration blob ({len(blob)} bytes)")


def decode_depth(blob: bytes) -> np.ndarray:
    """Data.depth -> float32 metres (0 where unknown)."""
    from PIL import Image
    im = Image.open(io.BytesIO(blob))
    a = np.asarray(im)
    if a.ndim == 3 and a.shape[2] == 4 and a.dtype == np.uint8:
        # float32 bit-cast as BGRA (module docstring).
        d = np.ascontiguousarray(a[..., [2, 1, 0, 3]]).view("<f4")[..., 0].astype(np.float32)
    elif a.ndim == 2 and a.dtype in (np.uint16, np.int32, np.uint32):
        d = a.astype(np.float32) * 0.001
    elif im.mode in ("I;16", "I;16B", "I"):
        d = np.asarray(im, dtype=np.float32) * 0.001
    else:
        raise DbError(f"unrecognised depth image (mode {im.mode}, shape {a.shape})")
    d[~np.isfinite(d) | (d < 0)] = 0.0
    return d


def depth_png_mm(depth_m: np.ndarray) -> bytes:
    """float metres -> 16-bit PNG millimetres (dataset's encoding). >65.5 m -> 0."""
    from PIL import Image
    mm = np.round(depth_m * 1000.0)
    mm[(mm < 0) | (mm > 65535)] = 0
    buf = io.BytesIO()
    Image.fromarray(mm.astype(np.uint16)).save(buf, format="PNG")
    return buf.getvalue()


def calib_yaml(node_id: int, c: dict, width: int, height: int) -> str:
    """What dataset.parse_calib reads; the layout rtabmap-export writes."""
    lt = " ".join(f"{v:.9g}," for v in c["local"][:3, :].ravel()).rstrip(",")
    return (
        "%YAML:1.0\n---\n"
        f'camera_name: "{node_id}"\n'
        f"image_width: {width}\nimage_height: {height}\n"
        "camera_matrix:\n   rows: 3\n   cols: 3\n"
        f"   data: [ {c['fx']:.17g}, 0., {c['cx']:.17g}, 0., {c['fy']:.17g}, {c['cy']:.17g}, 0., 0., 1. ]\n"
        "distortion_coefficients:\n   rows: 1\n   cols: 5\n   data: [ 0., 0., 0., 0., 0. ]\n"
        "distortion_model: plumb_bob\n"
        f"local_transform:\n   rows: 3\n   cols: 4\n   data: [ {lt} ]\n"
    )


# --------------------------------------------------------------------------
# The pose graph
# --------------------------------------------------------------------------

def connected_component(node_ids, links, seed: int) -> set[int]:
    adj = defaultdict(list)
    for l in links:
        adj[l["from"]].append(l["to"])
        adj[l["to"]].append(l["from"])
    keep, todo = {seed}, deque([seed])
    while todo:
        n = todo.popleft()
        for m in adj[n]:
            if m in node_ids and m not in keep:
                keep.add(m)
                todo.append(m)
    return keep


def _edge_residuals(Ri, ti, Rj, tj, Zinv_R, Zinv_t):
    """e = [t; log R] of Z^-1 T_i^-1 T_j, batched. Zero when T_j = T_i Z exactly."""
    RiT = np.transpose(Ri, (0, 2, 1))
    Rrel = RiT @ Rj
    trel = np.einsum("nij,nj->ni", RiT, tj - ti)
    Re = Zinv_R @ Rrel
    te = np.einsum("nij,nj->ni", Zinv_R, trel) + Zinv_t
    return np.concatenate([te, so3_log(Re)], axis=1)


def _sqrt_info(I: np.ndarray) -> np.ndarray:
    """L with L^T L = I, for whitening a residual. PSD-safe."""
    I = (I + I.T) / 2
    w, V = np.linalg.eigh(I)
    return (V * np.sqrt(np.clip(w, 0, None))).T


def optimize_graph(poses: dict[int, np.ndarray], links: list[dict], root: int,
                   iters: int = 30, tol: float = 1e-8, log=None) -> dict[int, np.ndarray]:
    """Gauss-Newton pose-graph optimization, `root` held fixed.

    Poses are perturbed on the right, per pose 6 dims (translation in the body frame,
    then a rotation vector), and each edge's 6x12 Jacobian is taken numerically for
    all edges at once: 24 batched residual evaluations an iteration, no hand-derived
    SE(3) Jacobians to get subtly wrong. Uses scipy's sparse solver when it is there
    (the `splat` env), a dense solve otherwise (fine below a few hundred nodes, which
    is all the system-python tests build).
    """
    ids = sorted(poses)
    if not links or len(ids) < 2:
        return {k: v.copy() for k, v in poses.items()}
    col = {nid: k for k, nid in enumerate(i for i in ids if i != root)}
    R = np.stack([poses[i][:3, :3] for i in ids])
    t = np.stack([poses[i][:3, 3] for i in ids])
    pos = {nid: k for k, nid in enumerate(ids)}
    ei = np.array([pos[l["from"]] for l in links])
    ej = np.array([pos[l["to"]] for l in links])
    Zinv = np.stack([np.linalg.inv(l["T"]) for l in links])
    Zinv_R, Zinv_t = Zinv[:, :3, :3], Zinv[:, :3, 3]
    L = np.stack([_sqrt_info(l["info"]) for l in links])

    try:
        import scipy.sparse as sp
        from scipy.sparse.linalg import spsolve
    except ImportError:
        sp = None

    n = 6 * len(col)
    eps = 1e-6
    prev = None
    for it in range(iters):
        e = _edge_residuals(R[ei], t[ei], R[ej], t[ej], Zinv_R, Zinv_t)
        r = np.einsum("nij,nj->ni", L, e)
        cost = float((r ** 2).sum())
        if prev is not None and abs(prev - cost) <= tol * max(prev, 1.0):
            break
        prev = cost
        # Numerical 6x12 Jacobian per edge, central differences, with the same
        # right perturbation the update below applies: t += R d_t, R = R Exp(d_r).
        J = np.zeros((len(links), 6, 12))
        base = ((R[ei], t[ei]), (R[ej], t[ej]))
        for side in (0, 1):
            Rb, tb = base[side]
            for d in range(6):
                for sgn in (1.0, -1.0):
                    dv = np.zeros(6)
                    dv[d] = sgn * eps
                    p = [list(base[0]), list(base[1])]
                    p[side] = [Rb @ so3_exp(dv[None, 3:])[0], tb + Rb @ dv[:3]]
                    ep = _edge_residuals(p[0][0], p[0][1], p[1][0], p[1][1], Zinv_R, Zinv_t)
                    J[:, :, side * 6 + d] += sgn * ep / (2 * eps)
        Jw = L @ J  # whitened
        # Free-variable column of each edge end, -1 for the fixed root.
        cix = np.array([col.get(nid, -1) for nid in ids])
        ends = (cix[ei], cix[ej])
        g = np.zeros(n)
        rows, cols, vals = [], [], []
        ar = np.arange(6)
        for a in (0, 1):
            Ja = Jw[:, :, 6 * a:6 * a + 6]
            ok = ends[a] >= 0
            np.add.at(g, (6 * ends[a][ok, None] + ar).ravel(),
                      np.einsum("nki,nk->ni", Ja[ok], r[ok]).ravel())
            for b in (0, 1):
                Jb = Jw[:, :, 6 * b:6 * b + 6]
                ok = (ends[a] >= 0) & (ends[b] >= 0)
                H = np.einsum("nki,nkj->nij", Ja[ok], Jb[ok])
                rows.append(np.broadcast_to(6 * ends[a][ok, None, None] + ar[None, :, None],
                                            H.shape).ravel())
                cols.append(np.broadcast_to(6 * ends[b][ok, None, None] + ar[None, None, :],
                                            H.shape).ravel())
                vals.append(H.ravel())
        rows, cols, vals = np.concatenate(rows), np.concatenate(cols), np.concatenate(vals)
        # A whisker of damping keeps a weakly-constrained direction (e.g. a pose only a
        # rotation-free link touches) from making the system singular.
        damp = 1e-9 * (np.abs(vals).max() if len(vals) else 1.0)
        if sp is not None:
            H = sp.coo_matrix((vals, (rows, cols)), shape=(n, n)).tocsc()
            H = H + sp.identity(n, format="csc") * damp
            dx = -spsolve(H, g)
        else:
            H = np.zeros((n, n))
            np.add.at(H, (rows, cols), vals)
            H[np.diag_indices(n)] += damp
            dx = -np.linalg.solve(H, g)
        for nid, c in col.items():
            k = pos[nid]
            d = dx[6 * c:6 * c + 6]
            t[k] = t[k] + R[k] @ d[:3]
            R[k] = R[k] @ so3_exp(d[None, 3:])[0]
        if log:
            log(f"  pose graph iter {it}: cost {cost:.6g}, |dx| {np.abs(dx).max():.3g}")
        if np.abs(dx).max() < 1e-9:
            break
    out = {}
    for nid in ids:
        T = np.eye(4)
        T[:3, :3], T[:3, 3] = R[pos[nid]], t[pos[nid]]
        out[nid] = T
    return out


def initial_guess(nodes: dict[int, dict], links: list[dict], root: int,
                  keep: set[int]) -> dict[int, np.ndarray]:
    """Chain link transforms out from the root (breadth first, neighbours first).

    Odometry poses would do for one session, but each session RTAB-Map merges starts
    its odometry again at the origin; composing links puts every node in the root's
    frame whatever session it came from.
    """
    # 0-1 BFS: a neighbour hop costs 0, a closure hop 1, so every node is placed by the
    # chain with the fewest closures in it. A plain BFS placed a node through a closure
    # whenever the closure was the shorter walk from the root (node 6 straight from node
    # 1 instead of along 1-2-...-6), so the "odometry" candidate graph_poses() scores
    # was partly closure-built (found 2026-11-17 by test_auto_keeps_odometry...).
    adj = defaultdict(list)
    for l in links:
        c = 0 if l["type"] == NEIGHBOR else 1
        adj[l["from"]].append((c, l["to"], l["T"]))
        adj[l["to"]].append((c, l["from"], np.linalg.inv(l["T"])))
    est = {root: nodes[root]["pose"].copy()}
    cost = {root: 0}
    done = set()
    todo = deque([root])
    while todo:
        a = todo.popleft()
        if a in done:
            continue
        done.add(a)
        for c, b, T in adj[a]:
            if b not in keep or b in done:
                continue
            if cost[a] + c < cost.get(b, math.inf):
                cost[b] = cost[a] + c
                est[b] = est[a] @ T
                (todo.appendleft if c == 0 else todo.append)(b)
    return est


def optimized_poses(con, log=None) -> tuple[dict[int, np.ndarray], dict[int, dict]]:
    """{node_id: 4x4 base_link pose in <rid>/map} after the loop-closure solve, plus the
    node table. The graph's own answer, whatever its consistency: graph_poses()
    is what the export uses."""
    poses, nodes, _ = graph_poses(con, log=log, source="optimized")
    return poses, nodes


# --------------------------------------------------------------------------
# Which poses: optimized or odometry
# --------------------------------------------------------------------------
#
# Neither wins everywhere, so the export measures both. On
# maps/fleet_warehouse_test (Isaac: near-perfect odometry, 2026-11-17, 105
# keyframe pairs, full-resolution targets) the odometry poses agree with each
# other's depth to 0.60 cm median, 99% of points within 5 cm; the optimized graph
# to 3.81 cm and 53%, because its visual loop
# closures sit ~10 cm (up to 29 cm) off the true motion and their information
# matrices (sigma ~5 mm) outweigh odometry's (3 cm). rtabmap-export follows the
# same closures. On the real robot it is the other way round: visual odometry
# drifts and the closures are what pull it back. A splat trained on cameras that
# disagree smears every surface they share, so `auto` keeps whichever pose set is
# more consistent across overlapping keyframes.

POSE_SOURCES = ("auto", "optimized", "odometry")
CONSISTENCY_DECIMATE = 6       # source pixels per check sample: 141x80 on a D435 frame
CONSISTENCY_MAX_PAIRS = 400
CONSISTENCY_INLIER_M = 0.05


class _DepthCache:
    """Full-resolution depth for the consistency check, a few frames at a time.
    Check pairs are mostly graph neighbours, so a small LRU hits; holding every
    frame (1.6 MB each as float32) would not fit a 1,157-keyframe map in RAM."""

    def __init__(self, con, size: int = 48):
        self.con, self.size, self.d = con, size, {}

    def get(self, nid):
        if nid in self.d:
            self.d[nid] = self.d.pop(nid)      # most recently used last
            return self.d[nid]
        row = self.con.execute("SELECT depth, calibration FROM Data WHERE id=?",
                               (nid,)).fetchone()
        val = None
        if row and row[0] and row[1]:
            try:
                c = parse_calibration(row[1])
                if c.get("type", 0) == 0:
                    depth = decode_depth(row[0])
                    h, w = depth.shape
                    cw, ch = (c["width"] or w), (c["height"] or h)
                    # intrinsics at the DEPTH image's size (it may be stored decimated)
                    val = (depth, {"fx": c["fx"] * w / cw, "fy": c["fy"] * h / ch,
                                   "cx": c["cx"] * w / cw, "cy": c["cy"] * h / ch,
                                   "local": c["local"]})
            except (DbError, OSError):
                val = None
        self.d[nid] = val
        while len(self.d) > self.size:
            self.d.pop(next(iter(self.d)))
        return val


def pair_consistency(cache: _DepthCache, P: dict, a: int, b: int):
    """Reproject every CONSISTENCY_DECIMATE-th pixel of a's depth into b's
    FULL-resolution depth under poses P: (median |dz| m, inlier fraction), or
    None when they overlap too little to say. b is never decimated: a target a
    few px off reads, on floor seen at a grazing angle, as centimetres of
    disagreement that isn't there (measured: 1.4 cm became 6.9 cm)."""
    fa, fb = cache.get(a), cache.get(b)
    if fa is None or fb is None:
        return None
    (da, ca), (db_, cb) = fa, fb
    s = CONSISTENCY_DECIMATE
    v, u = np.mgrid[0:da.shape[0]:s, 0:da.shape[1]:s]
    z = da[v, u]
    ok = (z > CLOUD_DEPTH_MIN) & (z < CLOUD_DEPTH_MAX)
    u, v, z = u[ok], v[ok], z[ok].astype(np.float64)
    pa = np.stack([(u - ca["cx"]) / ca["fx"] * z, (v - ca["cy"]) / ca["fy"] * z, z], axis=1)
    T = np.linalg.inv(P[b] @ cb["local"]) @ (P[a] @ ca["local"])
    q = pa @ T[:3, :3].T + T[:3, 3]
    zq = q[:, 2]
    ok = zq > CLOUD_DEPTH_MIN
    ub = np.rint(q[ok, 0] / zq[ok] * cb["fx"] + cb["cx"]).astype(int)
    vb = np.rint(q[ok, 1] / zq[ok] * cb["fy"] + cb["cy"]).astype(int)
    inb = (ub >= 0) & (ub < db_.shape[1]) & (vb >= 0) & (vb < db_.shape[0])
    zb, za = db_[vb[inb], ub[inb]], zq[ok][inb]
    good = (zb > CLOUD_DEPTH_MIN) & (zb < CLOUD_DEPTH_MAX) & (za < CLOUD_DEPTH_MAX)
    if good.sum() < 200:
        return None
    r = np.abs(za[good] - zb[good])
    return float(np.median(r)), float((r < CONSISTENCY_INLIER_M).mean())


def check_pairs(links: list[dict]) -> list[tuple[int, int]]:
    """Every loop closure (where the two pose sets differ most) plus an even
    sample of neighbour links, capped. Deterministic: same map, same choice."""
    closures = [(l["from"], l["to"]) for l in links if l["type"] != NEIGHBOR]
    nbrs = [(l["from"], l["to"]) for l in links if l["type"] == NEIGHBOR]
    room = max(CONSISTENCY_MAX_PAIRS - len(closures), CONSISTENCY_MAX_PAIRS // 4)
    if len(nbrs) > room:
        nbrs = nbrs[::math.ceil(len(nbrs) / room)]
    pairs = closures + nbrs
    if len(pairs) > CONSISTENCY_MAX_PAIRS:
        pairs = pairs[::math.ceil(len(pairs) / CONSISTENCY_MAX_PAIRS)]
    return pairs


def consistency(cache: _DepthCache, candidates: dict[str, dict], pairs) -> dict:
    """{name: {pairs, median_dz_m, inliers} | None}, scored on the pairs every
    candidate could score, so they're compared like for like."""
    rows = []
    for a, b in pairs:
        r = {name: pair_consistency(cache, P, a, b) for name, P in candidates.items()}
        if all(r.values()):
            rows.append(r)
    out = {}
    for name in candidates:
        if not rows:
            out[name] = None
            continue
        e = np.array([r[name] for r in rows])
        out[name] = {"pairs": len(e), "median_dz_m": float(np.median(e[:, 0])),
                     "inliers": float(e[:, 1].mean())}
    return out


def graph_poses(con, log=None, source: str = "auto"):
    """(base_link poses in <rid>/map, node table, summary). `source` picks the
    pose set (see the block comment above); `auto` measures both."""
    if source not in POSE_SOURCES:
        raise DbError(f"poses {source!r}: expected one of {', '.join(POSE_SOURCES)}")
    log = log or (lambda m: None)
    nodes = read_nodes(con)
    if not nodes:
        raise DbError("database has no nodes")
    links = [l for l in read_links(con) if l["from"] in nodes and l["to"] in nodes]
    keep = connected_component(set(nodes), links, max(nodes))
    root = min(keep)
    links = [l for l in links if l["from"] in keep and l["to"] in keep]
    n_loop = sum(1 for l in links if l["type"] != NEIGHBOR)
    log(f"pose graph: {len(keep)} nodes, {len(links)} links ({n_loop} loop closures)"
        + (f", {len(nodes) - len(keep)} unconnected nodes left out" if len(keep) < len(nodes) else ""))
    # Chained along neighbour links first: for one session this IS the odometry.
    odom = initial_guess(nodes, links, root, keep)
    summary = {"nodes": len(keep), "links": len(links), "loop_closures": n_loop}
    if source == "odometry" or (source == "auto" and not n_loop):
        summary["poses"] = "odometry" if source == "odometry" else "optimized"
        return odom, nodes, summary     # no closure: the optimum IS the chain
    opt = optimize_graph(odom, links, root)
    if source == "optimized":
        summary["poses"] = "optimized"
        return opt, nodes, summary
    scores = consistency(_DepthCache(con), {"optimized": opt, "odometry": odom},
                         check_pairs(links))
    summary["consistency"] = scores
    for name, sc in scores.items():
        if sc:
            log(f"  {name:9s} poses: median depth disagreement {sc['median_dz_m'] * 100:.2f} cm, "
                f"{sc['inliers'] * 100:.0f}% within {CONSISTENCY_INLIER_M * 100:.0f} cm "
                f"({sc['pairs']} keyframe pairs)")
    o, d = scores["optimized"], scores["odometry"]
    pick = "odometry" if (o and d and d["inliers"] > o["inliers"]) else "optimized"
    log(f"  using {pick} poses (configs/splat.yaml dataset.poses forces one)")
    summary["poses"] = pick
    return (odom if pick == "odometry" else opt), nodes, summary


# --------------------------------------------------------------------------
# Export
# --------------------------------------------------------------------------

def export(db: Path, out_dir: Path, prefix: str, log=print, poses_source: str = "auto") -> dict:
    """The rtabmap-export layout splat/dataset.py reads. Returns a small summary."""
    from PIL import Image

    out_dir = Path(out_dir)
    dirs = {k: out_dir / f"{prefix}_{k}" for k in ("rgb", "depth", "calib")}
    for d in dirs.values():
        d.mkdir(parents=True, exist_ok=True)
    con = connect(db)
    try:
        poses, nodes, summary = graph_poses(con, log=log, source=poses_source)
        lines = ["#timestamp x y z qx qy qz qw id"]
        cloud_xyz, cloud_rgb, chunk_n = [], [], 0
        n_frames = n_depth = 0
        for nid in sorted(poses):
            row = con.execute("SELECT image, depth, calibration FROM Data WHERE id=?",
                              (nid,)).fetchone()
            if row is None or not row[0] or not row[2]:
                continue
            img_blob, depth_blob, calib_blob = row
            try:
                c = parse_calibration(calib_blob)
            except DbError as e:
                log(f"  node {nid}: {e}; skipped")
                continue
            rgb_im = Image.open(io.BytesIO(img_blob))
            w, h = rgb_im.size
            if img_blob[:3] == b"\xff\xd8\xff":
                (dirs["rgb"] / f"{nid}.jpg").write_bytes(img_blob)
            else:
                rgb_im.convert("RGB").save(dirs["rgb"] / f"{nid}.jpg", quality=95)
            if c["width"] and (c["width"], c["height"]) != (w, h):
                # Mem/ImagePostDecimation shrinks the stored image, not the calibration.
                s = w / c["width"]
                c = {**c, "fx": c["fx"] * s, "fy": c["fy"] * s, "cx": c["cx"] * s,
                     "cy": c["cy"] * s}
            (dirs["calib"] / f"{nid}.yaml").write_text(calib_yaml(nid, c, w, h))
            cam = poses[nid] @ c["local"]
            qx, qy, qz, qw = rot_to_quat(cam[:3, :3])
            x, y, z = cam[:3, 3]
            lines.append(f"{nodes[nid]['stamp']:.6f} {x:.9g} {y:.9g} {z:.9g} "
                         f"{qx:.9g} {qy:.9g} {qz:.9g} {qw:.9g} {nid}")
            n_frames += 1
            if depth_blob and c.get("type", 0) == 0:
                depth = decode_depth(depth_blob)
                (dirs["depth"] / f"{nid}.png").write_bytes(depth_png_mm(depth))
                n_depth += 1
                xyz, rgb = backproject(depth, np.asarray(rgb_im.convert("RGB")), c, cam)
                cloud_xyz.append(xyz)
                cloud_rgb.append(rgb)
                chunk_n += len(xyz)
                if chunk_n > CLOUD_CHUNK_POINTS:
                    cloud_xyz, cloud_rgb = _voxel([np.concatenate(cloud_xyz)],
                                                  [np.concatenate(cloud_rgb)])
                    chunk_n = len(cloud_xyz[0])
    finally:
        con.close()
    if not n_frames:
        raise DbError(f"{db}: no node has an image and a calibration")
    (out_dir / f"{prefix}_camera_poses.txt").write_text("\n".join(lines) + "\n")
    n_points = 0
    if cloud_xyz:
        xyz, rgb = _voxel([np.concatenate(cloud_xyz)], [np.concatenate(cloud_rgb)])
        write_ply(out_dir / f"{prefix}_cloud.ply", xyz[0], rgb[0])
        n_points = len(xyz[0])
    log(f"exported {n_frames} keyframes ({n_depth} with depth), {n_points} cloud points")
    return {**summary, "frames": n_frames, "depth": n_depth, "points": n_points}


def backproject(depth: np.ndarray, rgb: np.ndarray, c: dict, cam: np.ndarray):
    """Every CLOUD_DECIMATION-th depth pixel -> (xyz in <rid>/map, rgb)."""
    dh, dw = depth.shape
    ih, iw = rgb.shape[:2]
    sx, sy = dw / iw, dh / ih   # depth may be stored smaller than the image
    v, u = np.mgrid[0:dh:CLOUD_DECIMATION, 0:dw:CLOUD_DECIMATION]
    z = depth[v, u]
    ok = (z > CLOUD_DEPTH_MIN) & (z < CLOUD_DEPTH_MAX)
    u, v, z = u[ok].astype(np.float64), v[ok].astype(np.float64), z[ok].astype(np.float64)
    x = (u - c["cx"] * sx) / (c["fx"] * sx) * z
    y = (v - c["cy"] * sy) / (c["fy"] * sy) * z
    pts = np.stack([x, y, z], axis=1) @ cam[:3, :3].T + cam[:3, 3]
    ui = np.clip((u / sx).astype(int), 0, iw - 1)
    vi = np.clip((v / sy).astype(int), 0, ih - 1)
    return pts.astype(np.float32), rgb[vi, ui].astype(np.uint8)


def _voxel(xyzs, rgbs):
    """One point per CLOUD_VOXEL cube (the first seen). dataset.build_dataset applies
    its own point budget later; this only keeps the running cloud from growing with
    every overlapping keyframe."""
    xyz, rgb = xyzs[0], rgbs[0]
    if not len(xyz):
        return [xyz], [rgb]
    lo = xyz.min(axis=0)
    keys = ((xyz - lo) / CLOUD_VOXEL).astype(np.int64)
    dims = np.maximum(keys.max(axis=0) + 1, 1)
    packed = (keys[:, 0] * dims[1] + keys[:, 1]) * dims[2] + keys[:, 2]
    idx = np.sort(np.unique(packed, return_index=True)[1])
    return [xyz[idx]], [rgb[idx]]


def write_ply(path: Path, xyz: np.ndarray, rgb: np.ndarray) -> None:
    rec = np.empty(len(xyz), dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
                                    ("red", "u1"), ("green", "u1"), ("blue", "u1")])
    rec["x"], rec["y"], rec["z"] = xyz.T
    rec["red"], rec["green"], rec["blue"] = rgb.T
    with open(path, "wb") as f:
        f.write(("ply\nformat binary_little_endian 1.0\n"
                 f"element vertex {len(xyz)}\n"
                 "property float x\nproperty float y\nproperty float z\n"
                 "property uchar red\nproperty uchar green\nproperty uchar blue\n"
                 "end_header\n").encode("ascii"))
        rec.tofile(f)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("db", type=Path)
    ap.add_argument("out_dir", type=Path, nargs="?")
    ap.add_argument("--prefix", default="robot_0")
    ap.add_argument("--poses", choices=POSE_SOURCES, default="auto",
                    help="pose set to export (default: whichever is more self-consistent)")
    ap.add_argument("--check", action="store_true",
                    help="pick the poses and report why; write nothing")
    a = ap.parse_args(argv)
    try:
        if a.check:
            con = connect(a.db)
            _, _, summary = graph_poses(con, log=print, source=a.poses)
            print(json.dumps(summary, indent=1))
            return 0
        if a.out_dir is None:
            ap.error("out_dir is required unless --check")
        export(a.db, a.out_dir, a.prefix, poses_source=a.poses)
    except DbError as e:
        print(f"rtabmap_db: {e}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
