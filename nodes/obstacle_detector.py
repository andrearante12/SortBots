#!/usr/bin/env python3
"""Geometric obstacle detector: head depth -> clustered obstacles in base_link.

Where `dynamic_obstacle_filter.py` answers "which pixels CHANGED since the last
frame" (to keep movers out of SLAM), this node answers "what is physically in
front of the robot right now, and is it moving". Frame differencing alone can't:
a person who stops walking vanishes from it, and the robot turning makes every
pixel change at once. Here every frame stands on its own geometry, and motion
comes from tracking cluster centroids in the odom frame, where the robot's own
motion has been taken out.

Pipeline (docs/obstacle_detection.md has the rationale and the tuning notes):

    camera/depth --back-project (strided)--> camera-frame points
      --TF--> base_link --range crop--> floor estimate --height band-->
      2-D cell grid --8-connected components--> clusters
      --> per-cluster stats (centroid, bbox, image box)
      --TF odom--> nearest-neighbour tracks --> velocity, `moving`

Outputs, all stamped with the source depth image's time:

    /<id>/obstacle_tracks  std_msgs/String JSON  (dashboard overlay, tests)
    /<id>/obstacle_points  sensor_msgs/PointCloud2 in <id>/base_link (Nav2-ready)
    /<id>/obstacles_viz    visualization_msgs/MarkerArray (RViz)

No class labels on purpose: Nav2 needs to know where something is and whether
it moves, not what it is, and geometry can't miss a class it wasn't trained on.

    python3 nodes/obstacle_detector.py --robot-id robot_0
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

import numpy as np
import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_CONFIG = REPO_ROOT / "configs" / "obstacle_detector.yaml"

# Every key here is documented in configs/obstacle_detector.yaml, which may
# override any of them.
DEFAULTS = {
    "rate_hz": 10.0,
    "stride": 4,
    "min_range_m": 0.3,
    "max_range_m": 4.5,
    "floor_z_m": 0.0,
    "floor_auto": True,
    "floor_search_m": 0.5,
    "floor_bin_m": 0.02,
    "floor_min_points": 200,
    "floor_smoothing": 0.2,
    "min_height_m": 0.08,
    "max_height_m": 2.0,
    "cell_m": 0.10,
    "min_cell_points": 3,
    "min_cluster_points": 30,
    "min_cluster_height_m": 0.10,
    "track_gate_m": 0.6,
    "track_max_age_s": 1.0,
    "track_min_hits": 5,
    "velocity_smoothing": 0.4,
    "moving_speed_mps": 0.25,
    "max_moving_extent_m": 1.5,
    "edge_margin_px": 8,
    "min_moving_z_extent_m": 0.3,
    "range_margin_m": 0.3,
    "max_obstacle_speed_mps": 3.0,
}


def load_config(path: Path | None) -> dict:
    cfg = dict(DEFAULTS)
    if path and path.exists():
        with open(path) as f:
            raw = yaml.safe_load(f) or {}
        cfg.update(raw)
    return cfg


def depth_to_metres(data, encoding: str, h: int, w: int):
    """Raw Image payload -> float32 metres, or None for an unsupported encoding.

    Sim publishes 32FC1 metres, the real D435 16UC1 millimetres with 0 = no
    return (which the range crop then drops).
    """
    if encoding == "32FC1":
        return np.frombuffer(data, dtype=np.float32).reshape(h, w)
    if encoding in ("16UC1", "mono16"):
        raw = np.frombuffer(data, dtype=np.uint16).reshape(h, w)
        return raw.astype(np.float32) * 0.001
    return None


def backproject(depth: np.ndarray, fx: float, fy: float, cx: float, cy: float,
                *, stride: int = 4, min_range_m: float = 0.3,
                max_range_m: float = 4.5):
    """Strided pinhole back-projection.

    Returns (Nx3 float32 points in the image's optical frame — x right, y
    down, z forward — and Nx2 int32 (u, v) pixel coordinates in the FULL image,
    so image boxes stay valid whatever the stride).
    """
    d = depth[::stride, ::stride] if stride > 1 else depth
    vs, us = np.nonzero(np.isfinite(d) & (d >= min_range_m) & (d <= max_range_m))
    z = d[vs, us].astype(np.float32)
    u = (us * stride).astype(np.int32)
    v = (vs * stride).astype(np.int32)
    x = (u.astype(np.float32) - cx) * z / fx
    y = (v.astype(np.float32) - cy) * z / fy
    return np.stack([x, y, z], axis=1), np.stack([u, v], axis=1)


def quat_to_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ], dtype=np.float32)


def estimate_floor_z(z: np.ndarray, *, prior: float, search_m: float,
                     bin_m: float, min_points: int) -> float | None:
    """Height of the floor plane in base_link, or None if too little floor is seen.

    The floor is the one dense horizontal surface near the expected height, so
    a z-histogram of the points within ±search_m of `prior` peaks there. A
    person's legs or a box side spread over many bins and never out-vote it.
    Needed because base_link's origin isn't on the floor in every model (the
    sim's height comes from the USD and the real mount is still unmeasured).
    """
    near = z[np.abs(z - prior) <= search_m]
    if near.size < min_points:
        return None
    bins = max(1, int(math.ceil(2 * search_m / bin_m)))
    counts, edges = np.histogram(near, bins=bins,
                                 range=(prior - search_m, prior + search_m))
    k = int(np.argmax(counts))
    if counts[k] < min_points:
        return None
    in_bin = near[(near >= edges[k]) & (near <= edges[k + 1])]
    return float(np.median(in_bin))


def cluster_cells(xy: np.ndarray, *, cell_m: float, min_cell_points: int):
    """8-connected components over a 2-D occupancy grid of the points' x, y.

    Returns an int32 label per point, -1 for points in cells with fewer than
    `min_cell_points` (isolated depth speckle). Two points join a cluster when
    their cells touch, so the link distance is between cell_m and ~2.8·cell_m.
    numpy-only, a grid stand-in for DBSCAN: scipy and sklearn aren't repo
    dependencies, and on the Orin Nano a few hundred occupied cells is far
    cheaper to union than tens of thousands of points.
    """
    n = xy.shape[0]
    labels = np.full(n, -1, dtype=np.int32)
    if n == 0:
        return labels
    ij = np.floor(xy / cell_m).astype(np.int64)
    cells, inverse, counts = np.unique(ij, axis=0, return_inverse=True,
                                       return_counts=True)
    inverse = inverse.reshape(-1)
    keep = counts >= min_cell_points
    index = {(int(a), int(b)): k for k, (a, b) in enumerate(cells) if keep[k]}

    parent = list(range(len(cells)))

    def find(i: int) -> int:
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    # Half the 8-neighbourhood is enough: each pair gets visited from one side.
    for (a, b), k in index.items():
        for da, db in ((1, -1), (1, 0), (1, 1), (0, 1)):
            j = index.get((a + da, b + db))
            if j is not None:
                ra, rb = find(k), find(j)
                if ra != rb:
                    parent[max(ra, rb)] = min(ra, rb)

    cell_label = np.full(len(cells), -1, dtype=np.int32)
    remap: dict[int, int] = {}
    for k in index.values():
        root = find(k)
        cell_label[k] = remap.setdefault(root, len(remap))
    labels[:] = cell_label[inverse]
    return labels


def summarize_clusters(pts: np.ndarray, uv: np.ndarray, labels: np.ndarray,
                       *, floor_z: float, min_points: int,
                       min_height_m: float, width: int | None = None,
                       edge_margin_px: int = 8, depth: np.ndarray | None = None,
                       far_limit_m: float | None = None) -> list[dict]:
    """Per-cluster stats in base_link, nearest first.

    `height_m` is the top of the cluster above the floor, and `uv` the
    [u0, v0, u1, v1] box in image pixels that the dashboard draws. `clipped`
    means only part of the object is in view, so its centroid isn't where the
    object is: the box touches the image's left or right edge, or (given each
    point's camera `depth`) the cluster reaches `far_limit_m`, where the range
    crop cuts it.
    """
    out = []
    if labels.size == 0 or labels.max() < 0:
        return out
    for lab in range(int(labels.max()) + 1):
        m = labels == lab
        if int(m.sum()) < min_points:
            continue
        p = pts[m]
        lo, hi = p.min(axis=0), p.max(axis=0)
        height = float(hi[2] - floor_z)
        if height < min_height_m:
            continue
        c = p.mean(axis=0)
        # Range to the NEAREST point, not the centroid: that's the distance
        # that matters for stopping, and a long shelf's centroid can sit metres
        # past the end that's actually closest.
        rng = float(np.min(np.hypot(p[:, 0], p[:, 1])))
        q = uv[m]
        u0, u1 = int(q[:, 0].min()), int(q[:, 0].max())
        clipped = width is not None and (
            u0 <= edge_margin_px or u1 >= width - 1 - edge_margin_px)
        if depth is not None and far_limit_m is not None:
            clipped = clipped or float(depth[m].max()) >= far_limit_m
        out.append({
            "centroid": [round(float(v), 3) for v in c],
            "min": [round(float(v), 3) for v in lo],
            "max": [round(float(v), 3) for v in hi],
            "extent_m": round(float(max(hi[0] - lo[0], hi[1] - lo[1])), 3),
            "height_m": round(height, 3),
            "range_m": round(rng, 3),
            "bearing_deg": round(math.degrees(math.atan2(c[1], c[0])), 1),
            "n": int(m.sum()),
            "uv": [u0, int(q[:, 1].min()), u1, int(q[:, 1].max())],
            "clipped": bool(clipped),
            "label": lab,
        })
    out.sort(key=lambda o: o["range_m"])
    return out


def detect(depth: np.ndarray, K, R_base_cam: np.ndarray, t_base_cam: np.ndarray,
           cfg: dict, floor_prior: float | None = None):
    """One depth frame -> (obstacles, obstacle points Nx3 in base_link, floor_z).

    Pure: no ROS, no state. `K` is the 3x3 row-major intrinsics (CameraInfo.k).
    `floor_prior` is the last frame's floor estimate; on frames with too little
    floor in view it is reused, falling back to cfg["floor_z_m"].
    """
    c = {**DEFAULTS, **cfg}
    fx, fy, cx, cy = float(K[0]), float(K[4]), float(K[2]), float(K[5])
    cam, uv = backproject(depth, fx, fy, cx, cy, stride=int(c["stride"]),
                          min_range_m=c["min_range_m"],
                          max_range_m=c["max_range_m"])
    pts = cam @ R_base_cam.T + t_base_cam

    prior = c["floor_z_m"] if floor_prior is None else floor_prior
    floor_z = prior
    if c["floor_auto"]:
        est = estimate_floor_z(pts[:, 2], prior=c["floor_z_m"],
                               search_m=c["floor_search_m"],
                               bin_m=c["floor_bin_m"],
                               min_points=int(c["floor_min_points"]))
        if est is not None:
            floor_z = est

    band = ((pts[:, 2] >= floor_z + c["min_height_m"])
            & (pts[:, 2] <= floor_z + c["max_height_m"]))
    pts, uv, cam_z = pts[band], uv[band], cam[band, 2]
    labels = cluster_cells(pts[:, :2], cell_m=c["cell_m"],
                           min_cell_points=int(c["min_cell_points"]))
    obstacles = summarize_clusters(pts, uv, labels, floor_z=floor_z,
                                   min_points=int(c["min_cluster_points"]),
                                   min_height_m=c["min_cluster_height_m"],
                                   width=depth.shape[1],
                                   edge_margin_px=int(c["edge_margin_px"]),
                                   depth=cam_z,
                                   far_limit_m=c["max_range_m"] - c["range_margin_m"])
    kept = {o["label"] for o in obstacles}
    in_obstacle = np.isin(labels, list(kept)) if kept else np.zeros(len(labels), bool)
    for o in obstacles:
        del o["label"]
    return obstacles, pts[in_obstacle], floor_z


def can_move(o: dict, cfg: dict | None = None) -> bool:
    """Whether an obstacle's centroid motion can be trusted as real motion.

    Each rule removes a way a STATIC object's centroid slides while the robot
    turns, measured live in sim on 2026-10-22 (turning in place at 0.5 rad/s):

    - clipped: cut off at the image edge, more of it slides into view each
      frame. 41 of 170 frames had a false "moving" before this rule. Also cut
      off at the far end of the range crop: driving toward a thin upright at
      ~4.3 m pulled more of it inside max_range_m each frame (42 -> 134 points)
      and read as 0.3-0.7 m/s in 20 of 331 frames (2-robot run, 2026-10-22).
    - long (extent_m): shelving and walls do the same along their length.
    - flat (z extent): a shelf deck near camera height is seen edge-on as a
      ~16 px line whose visible stretch shifts with the view angle. That was
      the remaining 11 of 171. A body is tall top to bottom; a deck isn't.
    """
    c = {**DEFAULTS, **(cfg or {})}
    return (not o.get("clipped", False)
            and o["extent_m"] <= c["max_moving_extent_m"]
            and o["max"][2] - o["min"][2] >= c["min_moving_z_extent_m"])


class Tracker:
    """Nearest-neighbour tracks on obstacle centroids in a world-fixed frame.

    v2 in docs/obstacle_detection.md is a Kalman filter; this is the
    smallest thing that gives a usable `moving` flag. Positions MUST be in a
    frame that doesn't move with the robot (odom), or turning in place would
    make every shelf "move". Times are message stamps, so sim time works.
    """

    def __init__(self, cfg: dict | None = None):
        self.cfg = {**DEFAULTS, **(cfg or {})}
        self.tracks: dict[int, dict] = {}
        self._next_id = 1

    def update(self, positions: list[tuple[float, float]], t: float,
               movable: list[bool] | None = None) -> list[dict]:
        """Associate this frame's centroids; returns one track state per input.

        An observation that isn't `movable` (see can_move) still moves the
        track, but its velocity is zeroed rather than fed a centroid slide. So
        a person walking into view is called moving a couple of frames after
        they're fully in it, not while they're clipped by the edge.
        """
        movable = movable if movable is not None else [True] * len(positions)
        c = self.cfg
        for tid in [k for k, tr in self.tracks.items()
                    if t - tr["t"] > c["track_max_age_s"]]:
            del self.tracks[tid]

        # Greedy by distance: pair the closest (track, detection) first.
        pairs = []
        for tid, tr in self.tracks.items():
            dt = max(t - tr["t"], 0.0)
            px = tr["pos"][0] + tr["vel"][0] * dt
            py = tr["pos"][1] + tr["vel"][1] * dt
            for i, (x, y) in enumerate(positions):
                d = math.hypot(x - px, y - py)
                if d <= c["track_gate_m"]:
                    pairs.append((d, tid, i))
        pairs.sort()
        taken_t, assigned = set(), {}
        for _, tid, i in pairs:
            if tid in taken_t or i in assigned:
                continue
            taken_t.add(tid)
            assigned[i] = tid

        out = []
        a = c["velocity_smoothing"]
        for i, (x, y) in enumerate(positions):
            tid = assigned.get(i)
            if tid is None:
                tid = self._next_id
                self._next_id += 1
                self.tracks[tid] = {"pos": (x, y), "vel": (0.0, 0.0), "t": t,
                                    "hits": 1}
            else:
                tr = self.tracks[tid]
                dt = t - tr["t"]
                vx = vy = 0.0
                if dt > 1e-3:
                    vx = (x - tr["pos"][0]) / dt
                    vy = (y - tr["pos"][1]) / dt
                if math.hypot(vx, vy) > c["max_obstacle_speed_mps"]:
                    # Faster than anything on a warehouse floor: the cluster
                    # split or merged and its centroid jumped (live, 2026-10-22:
                    # a shelf track 24 frames old read 1.2 m/s for a frame while
                    # turning). Sim frames are ~0.03 s apart, so even a 0.1 m
                    # jump inside the gate is 3 m/s. Start the track over: it
                    # has to earn track_min_hits again before it can move.
                    tr["vel"], tr["hits"] = (0.0, 0.0), 0
                elif not movable[i]:
                    tr["vel"] = (0.0, 0.0)
                elif dt > 1e-3:
                    tr["vel"] = (a * vx + (1 - a) * tr["vel"][0],
                                 a * vy + (1 - a) * tr["vel"][1])
                tr["pos"], tr["t"] = (x, y), t
                tr["hits"] += 1
            tr = self.tracks[tid]
            speed = math.hypot(*tr["vel"])
            moving = (movable[i]
                      and tr["hits"] >= c["track_min_hits"]
                      and speed >= c["moving_speed_mps"])
            out.append({"track_id": tid,
                        "vel": [round(tr["vel"][0], 3), round(tr["vel"][1], 3)],
                        "speed_mps": round(speed, 3),
                        "moving": bool(moving),
                        "hits": tr["hits"]})
        return out


def build_payload(obstacles: list[dict], *, stamp: float, frame: str,
                  floor_z: float, width: int, height: int,
                  world_frame: str | None) -> dict:
    return {
        "stamp": round(float(stamp), 3),
        "frame": frame,
        "world_frame": world_frame,
        "floor_z": round(float(floor_z), 3),
        "w": int(width),
        "h": int(height),
        "count": len(obstacles),
        "moving": sum(1 for o in obstacles if o.get("moving")),
        "obstacles": obstacles,
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-id", default="robot_0")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    args, ros_args = parser.parse_known_args(argv)
    cfg = load_config(args.config)

    import rclpy
    from rclpy.duration import Duration
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.time import Time
    from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
    from std_msgs.msg import String
    from tf2_ros import Buffer, TransformListener
    from visualization_msgs.msg import Marker, MarkerArray

    robot_id = args.robot_id
    base_frame = f"{robot_id}/base_link"
    odom_frame = f"{robot_id}/odom"

    def to_rt(tf):
        q, t = tf.transform.rotation, tf.transform.translation
        return (quat_to_matrix(q.x, q.y, q.z, q.w),
                np.array([t.x, t.y, t.z], dtype=np.float32))

    class ObstacleDetector(Node):
        def __init__(self):
            super().__init__(f"{robot_id}_obstacle_detector")
            self._info = None
            self._floor = None
            self._last_run = 0.0
            self._tracker = Tracker(cfg)
            self._tf = Buffer()
            self._tf_listener = TransformListener(self._tf, self)
            # sensor_data (BEST_EFFORT) on both camera topics, never RELIABLE:
            # a RELIABLE image subscriber kills Isaac's image writers at around
            # sim-t=100-140 s (CLAUDE.md).
            qos = qos_profile_sensor_data
            self.create_subscription(Image, f"/{robot_id}/camera/depth",
                                     self._on_depth, qos)
            self.create_subscription(CameraInfo, f"/{robot_id}/camera/camera_info",
                                     self._on_info, qos)
            self.json_pub = self.create_publisher(String, f"/{robot_id}/obstacle_tracks", 1)
            self.cloud_pub = self.create_publisher(
                PointCloud2, f"/{robot_id}/obstacle_points", qos)
            self.viz_pub = self.create_publisher(
                MarkerArray, f"/{robot_id}/obstacles_viz", 1)
            self._warned: set[str] = set()
            self.get_logger().info(
                f"obstacle detector: camera/depth -> obstacles, obstacle_points, "
                f"obstacles_viz for {robot_id} (config {args.config})")

        def _warn_once(self, key: str, text: str):
            if key not in self._warned:
                self._warned.add(key)
                self.get_logger().warn(text)

        def _on_info(self, msg):
            self._info = msg

        def _lookup(self, target: str, source: str, stamp):
            # At the image's stamp first; the latest transform if TF hasn't
            # caught up (a static mount makes the two the same anyway).
            for when in (Time.from_msg(stamp), Time()):
                try:
                    return self._tf.lookup_transform(target, source, when,
                                                     timeout=Duration(seconds=0.0))
                except Exception:  # noqa: BLE001 — tf2 raises several types
                    continue
            return None

        def _on_depth(self, msg):
            # Throttled on the steady clock, not the node clock: this is a
            # CPU budget, not a sim-time deadline (CLAUDE.md, ROS timers).
            now = time.monotonic()
            if now - self._last_run < 1.0 / cfg["rate_hz"]:
                return
            if self._info is None:
                self._warn_once("info", "waiting for camera_info")
                return
            depth = depth_to_metres(msg.data, msg.encoding, msg.height, msg.width)
            if depth is None:
                self._warn_once("enc", f"unsupported depth encoding {msg.encoding}")
                return
            cam_tf = self._lookup(base_frame, msg.header.frame_id, msg.header.stamp)
            if cam_tf is None:
                self._warn_once("tf", f"no TF {base_frame} <- {msg.header.frame_id} yet")
                return
            self._last_run = now
            R, t = to_rt(cam_tf)
            obstacles, pts, floor_z = detect(depth, self._info.k, R, t, cfg,
                                             floor_prior=self._floor)
            a = cfg["floor_smoothing"]
            self._floor = floor_z if self._floor is None else (
                a * floor_z + (1 - a) * self._floor)

            stamp_s = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            odom_tf = self._lookup(odom_frame, base_frame, msg.header.stamp)
            world = odom_frame if odom_tf is not None else None
            if odom_tf is None:
                self._warn_once("odom", f"no TF {odom_frame} <- {base_frame}: "
                                "tracking in base_link, so `moving` is only "
                                "meaningful while the robot is still")
                Ro, to = np.eye(3, dtype=np.float32), np.zeros(3, np.float32)
            else:
                Ro, to = to_rt(odom_tf)
            positions = []
            for o in obstacles:
                w = Ro @ np.asarray(o["centroid"], np.float32) + to
                positions.append((float(w[0]), float(w[1])))
            states = self._tracker.update(
                positions, stamp_s, [can_move(o, cfg) for o in obstacles])
            for o, s in zip(obstacles, states):
                o.update(s)

            payload = build_payload(obstacles, stamp=stamp_s, frame=base_frame,
                                    floor_z=floor_z, width=msg.width,
                                    height=msg.height, world_frame=world)
            self.json_pub.publish(String(data=json.dumps(payload)))
            self.cloud_pub.publish(self._cloud(msg.header.stamp, pts))
            self.viz_pub.publish(self._markers(msg.header.stamp, obstacles))

        def _cloud(self, stamp, pts):
            out = PointCloud2()
            out.header.stamp = stamp
            out.header.frame_id = base_frame
            out.height, out.width = 1, int(pts.shape[0])
            out.fields = [PointField(name=n, offset=4 * i,
                                     datatype=PointField.FLOAT32, count=1)
                          for i, n in enumerate("xyz")]
            out.is_bigendian = False
            out.point_step = 12
            out.row_step = 12 * out.width
            out.is_dense = True
            out.data = pts.astype(np.float32).tobytes()
            return out

        def _markers(self, stamp, obstacles):
            arr = MarkerArray()
            clear = Marker()
            clear.action = Marker.DELETEALL
            arr.markers.append(clear)
            for i, o in enumerate(obstacles):
                lo, hi = o["min"], o["max"]
                box = Marker()
                box.header.stamp = stamp
                box.header.frame_id = base_frame
                box.ns, box.id = "obstacles", i
                box.type, box.action = Marker.CUBE, Marker.ADD
                box.pose.position.x = (lo[0] + hi[0]) / 2
                box.pose.position.y = (lo[1] + hi[1]) / 2
                box.pose.position.z = (lo[2] + hi[2]) / 2
                box.pose.orientation.w = 1.0
                box.scale.x = max(hi[0] - lo[0], 0.02)
                box.scale.y = max(hi[1] - lo[1], 0.02)
                box.scale.z = max(hi[2] - lo[2], 0.02)
                # Red = moving, amber = static; matches the dashboard overlay.
                box.color.r, box.color.g, box.color.b = (
                    (1.0, 0.2, 0.2) if o.get("moving") else (1.0, 0.7, 0.1))
                box.color.a = 0.45
                arr.markers.append(box)
                label = Marker()
                label.header = box.header
                label.ns, label.id = "obstacle_labels", i
                label.type, label.action = Marker.TEXT_VIEW_FACING, Marker.ADD
                label.pose.position.x = box.pose.position.x
                label.pose.position.y = box.pose.position.y
                label.pose.position.z = hi[2] + 0.15
                label.pose.orientation.w = 1.0
                label.scale.z = 0.15
                label.color.r = label.color.g = label.color.b = label.color.a = 1.0
                label.text = (f"#{o.get('track_id', '?')} {o['range_m']:.1f} m"
                              + (f" {o['speed_mps']:.1f} m/s" if o.get("moving") else ""))
                arr.markers.append(label)
            return arr

    rclpy.init(args=ros_args)
    node = ObstacleDetector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # How rclpy reports the SIGTERM teardown sends: a normal stop.
        pass
    except Exception:  # noqa: BLE001
        # The same SIGTERM can land mid-spin and surface as RCLError ("context
        # is not valid") instead; only a failure with the context still up is real.
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main(sys.argv[1:])
