"""splat/rtabmap_db.py: an RTAB-Map DB read without ROS. numpy + Pillow + sqlite3 only.

The synthetic DB has the 0.22 schema's columns the reader touches and the blob
encodings seen in a real 0.22.1 DB (2026-11-17): JPEG image, float32 depth bit-cast
into a BGRA PNG, the int32[11]-header calibration.
"""
import io
import math
import sqlite3
import struct
import sys
from pathlib import Path

import numpy as np
import pytest

PIL = pytest.importorskip("PIL")
from PIL import Image  # noqa: E402

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "splat"))
sys.path.insert(0, str(REPO / "nodes"))

import dataset  # noqa: E402
import rtabmap_db as R  # noqa: E402

W, H = 64, 48
FX = FY = 50.0
CX, CY = 32.0, 24.0
# base_link -> optical at 1.19 m: camera z = robot +x, camera x = robot -y.
LOCAL = np.array([[0, 0, 1, 0], [-1, 0, 0, 0], [0, -1, 0, 1.19], [0, 0, 0, 1]], float)


def pose(x, y, yaw):
    T = np.eye(4)
    c, s = math.cos(yaw), math.sin(yaw)
    T[:2, :2] = [[c, -s], [s, c]]
    T[:2, 3] = (x, y)
    return T


def blob34(T):
    return np.asarray(T[:3, :], "<f4").tobytes()


def calib_blob(w=W, h=H, f=FX):
    cx, cy = w / 2, h / 2
    hdr = struct.pack("<11i", 0, 22, 1, 0, w, h, 9, 5, 9, 12, 12)
    K = [f, 0, cx, 0, f, cy, 0, 0, 1]
    P = [f, 0, cx, 0, 0, f, cy, 0, 0, 0, 1, 0]
    return (hdr + struct.pack("<9d", *K) + struct.pack("<5d", *[0] * 5)
            + struct.pack("<9d", *np.eye(3).ravel()) + struct.pack("<12d", *P)
            + np.asarray(LOCAL[:3, :], "<f4").tobytes())


def depth_bgra_png(depth_m):
    """What cv::imencode makes of a 32FC1 Mat viewed as 8UC4: bytes stored as BGRA."""
    h, w = depth_m.shape
    b = np.ascontiguousarray(depth_m.astype("<f4")).view(np.uint8).reshape(h, w, 4)
    buf = io.BytesIO()
    Image.fromarray(b[..., [2, 1, 0, 3]], "RGBA").save(buf, format="PNG")
    return buf.getvalue()


def jpeg(color, w=W, h=H):
    buf = io.BytesIO()
    Image.new("RGB", (w, h), color).save(buf, format="JPEG")
    return buf.getvalue()


def make_db(path, odom, links, depth_m=2.0, null_links=(), depth_of=None, size=(W, H, FX)):
    """`depth_of(nid)` -> an (h, w) float32 depth image overrides the flat `depth_m`."""
    con = sqlite3.connect(path)
    con.executescript("""
        CREATE TABLE Node (id INTEGER PRIMARY KEY, map_id INTEGER, weight INTEGER,
                           stamp FLOAT, pose BLOB, ground_truth_pose BLOB);
        CREATE TABLE Data (id INTEGER PRIMARY KEY, image BLOB, depth BLOB, calibration BLOB);
        CREATE TABLE Link (from_id INTEGER, to_id INTEGER, type INTEGER,
                           information_matrix BLOB, transform BLOB);
    """)
    w, h, f = size
    for nid, T in odom.items():
        depth = depth_of(nid) if depth_of else np.full((h, w), depth_m, np.float32)
        con.execute("INSERT INTO Node VALUES (?,?,?,?,?,NULL)", (nid, 0, 0, float(nid), blob34(T)))
        con.execute("INSERT INTO Data VALUES (?,?,?,?)",
                    (nid, jpeg((10 * nid % 255, 80, 160), w, h), depth_bgra_png(depth),
                     calib_blob(w, h, f)))
    info = np.diag([1000.0] * 3 + [100.0] * 3).astype("<f8").tobytes()
    for f, t, typ, T in links:
        con.execute("INSERT INTO Link VALUES (?,?,?,?,?)", (f, t, typ, info, blob34(T)))
    for f, t in null_links:
        con.execute("INSERT INTO Link VALUES (?,?,?,?,?)",
                    (f, t, R.GLOBAL_CLOSURE, info, bytes(48)))
    con.commit()
    con.close()


def chain(odom):
    """Neighbour links that reproduce the odometry exactly, stored new -> old as RTAB-Map does."""
    ids = sorted(odom)
    return [(b, a, R.NEIGHBOR, np.linalg.inv(odom[b]) @ odom[a]) for a, b in zip(ids, ids[1:])]


def test_depth_decodes_bgra_float_and_16bit_mm():
    d = np.linspace(0.5, 7.5, W * H, dtype=np.float32).reshape(H, W)
    np.testing.assert_allclose(R.decode_depth(depth_bgra_png(d)), d, rtol=0, atol=1e-6)
    np.testing.assert_allclose(R.decode_depth(R.depth_png_mm(d)), d, atol=6e-4)


def test_calibration_blob():
    c = R.parse_calibration(calib_blob())
    assert (c["fx"], c["fy"], c["cx"], c["cy"], c["width"], c["height"]) == (FX, FY, CX, CY, W, H)
    np.testing.assert_allclose(c["local"], LOCAL, atol=1e-6)


def test_calib_yaml_is_what_dataset_reads():
    c = R.parse_calibration(calib_blob())
    got = dataset.parse_calib(R.calib_yaml(7, c, W, H))
    assert got == {"w": W, "h": H, "fl_x": FX, "fl_y": FY, "cx": CX, "cy": CY}


def test_quaternion_roundtrip():
    from recon_fuse import quat_to_rot
    for T in (pose(0, 0, 0.3) @ LOCAL, pose(1, 2, -2.9) @ LOCAL, LOCAL):
        np.testing.assert_allclose(quat_to_rot(*R.rot_to_quat(T[:3, :3])), T[:3, :3], atol=1e-9)


def test_export_without_closures_is_odometry(tmp_path):
    odom = {i: pose(0.5 * i, 0, 0.1 * i) for i in range(1, 6)}
    make_db(tmp_path / "m.db", odom, chain(odom))
    s = R.export(tmp_path / "m.db", tmp_path / "out", "robot_0", log=lambda m: None)
    assert s["frames"] == 5 and s["depth"] == 5 and s["points"] > 0
    frames, files = dataset.load_export(tmp_path / "out", "robot_0", "robot_0", np.eye(4))
    assert [f["id"] for f in frames] == [1, 2, 3, 4, 5]
    for f in frames:
        np.testing.assert_allclose(np.array(f["transform_matrix"]), odom[f["id"]] @ LOCAL, atol=1e-5)
        assert f["depth_file_path"]
    # The cloud: a wall 2 m in front of each camera, all at z in (0.3, 6) of the camera.
    v = dataset.read_ply_vertices(tmp_path / "out" / "robot_0_cloud.ply")
    xyz = np.stack([v["x"], v["y"], v["z"]], 1)
    assert np.all(np.abs(xyz[:, 2] - 1.19) < 2.0 * (CY / FY) + 1e-3)


def test_null_transform_links_are_skipped_and_their_node_dropped(tmp_path):
    odom = {i: pose(0.5 * i, 0, 0) for i in range(1, 5)}
    links = chain({k: odom[k] for k in (1, 3, 4)})  # node 2 joined only by a null link
    make_db(tmp_path / "m.db", odom, links, null_links=[(2, 1)])
    s = R.export(tmp_path / "m.db", tmp_path / "out", "robot_0", log=lambda m: None)
    assert s["frames"] == 3
    assert not (tmp_path / "out" / "robot_0_rgb" / "2.jpg").exists()


def test_loop_closure_removes_odometry_drift(tmp_path):
    # Ground truth: a 4 m square driven twice round (a node every 0.5 m), back to the start.
    gt = {}
    nid = 1
    for lap in range(2):
        for side in range(4):
            for k in range(8):
                yaw = side * math.pi / 2
                x0, y0 = [(0, 0), (4, 0), (4, 4), (0, 4)][side]
                gt[nid] = pose(x0 + 0.5 * k * math.cos(yaw), y0 + 0.5 * k * math.sin(yaw), yaw)
                nid += 1
    ids = sorted(gt)
    # Odometry drifts: every step's yaw is 1.5 deg too big, so the second lap is well off.
    odom = {ids[0]: gt[ids[0]]}
    for a, b in zip(ids, ids[1:]):
        step = np.linalg.inv(gt[a]) @ gt[b]
        odom[b] = odom[a] @ step @ pose(0, 0, math.radians(1.5))
    links = [(b, a, R.NEIGHBOR, np.linalg.inv(odom[b]) @ odom[a]) for a, b in zip(ids, ids[1:])]
    # Loop closures from "registration": exact, second lap node -> same place in lap one.
    for k in range(0, 32, 4):
        a, b = ids[k], ids[k + 32]
        links.append((b, a, R.GLOBAL_CLOSURE, np.linalg.inv(gt[b]) @ gt[a]))
    make_db(tmp_path / "m.db", odom, links)
    con = R.connect(tmp_path / "m.db")
    opt, _ = R.optimized_poses(con)
    con.close()

    def ate(P):
        return max(np.linalg.norm(P[i][:3, 3] - gt[i][:3, 3]) for i in ids)
    drift, fixed = ate(odom), ate(opt)
    assert drift > 1.0, drift          # the setup really drifts
    assert fixed < 0.25 * drift, (drift, fixed)
    # The root is held where it was.
    np.testing.assert_allclose(opt[ids[0]], odom[ids[0]], atol=1e-9)


def test_library_db_is_opened_read_only(tmp_path):
    odom = {i: pose(0.5 * i, 0, 0) for i in range(1, 3)}
    make_db(tmp_path / "m.db", odom, chain(odom))
    con = R.connect(tmp_path / "m.db")
    with pytest.raises(sqlite3.OperationalError):
        con.execute("DELETE FROM Node")
    con.close()


# -- which poses: optimized or odometry -----------------------------------------
#
# A wall at x = 3 m, robots sliding along y facing it; depth is rendered from the TRUE
# poses, so it is the referee. Big enough frames that every pair scores (>= 200 samples
# at CONSISTENCY_DECIMATE).
BW, BH, BF = 192, 144, 150.0
WALL_X = 3.0


def wall_depth(T_true):
    cam = T_true @ LOCAL
    # Facing +x with yaw 0, every pixel's optical z is along world +x: flat depth.
    assert np.allclose(cam[:3, 2], [1, 0, 0])
    return np.full((BH, BW), WALL_X - cam[0, 3], np.float32)


def slide(n, x_of=lambda i: 0.0):
    return {i: pose(x_of(i), 0.4 * (i - 1), 0.0) for i in range(1, n + 1)}


def pose_choice(tmp_path, truth, odom, closure_T):
    links = chain(odom) + [(max(odom), min(odom), R.GLOBAL_CLOSURE, closure_T)]
    make_db(tmp_path / "m.db", odom, links, depth_of=lambda i: wall_depth(truth[i]),
            size=(BW, BH, BF))
    con = R.connect(tmp_path / "m.db")
    try:
        poses, _, summary = R.graph_poses(con, source="auto")
    finally:
        con.close()
    return poses, summary


def test_auto_keeps_odometry_when_a_closure_is_wrong(tmp_path):
    # The sim's case (2026-11-17): odometry is right, a visual closure is 30 cm off.
    truth = slide(6)
    wrong = np.linalg.inv(truth[6]) @ truth[1] @ pose(0.3, 0, 0)
    poses, summary = pose_choice(tmp_path, truth, truth, wrong)
    assert summary["poses"] == "odometry", summary
    sc = summary["consistency"]
    assert sc["odometry"]["median_dz_m"] < 0.005 < sc["optimized"]["median_dz_m"], sc
    for i in truth:
        np.testing.assert_allclose(poses[i], truth[i], atol=1e-5)


def test_auto_takes_the_graph_when_odometry_drifts(tmp_path):
    # The real robot's case: visual odometry creeps 6 cm a step toward the wall, the
    # closure back to the start is right.
    truth = slide(6)
    odom = slide(6, x_of=lambda i: 0.06 * (i - 1))
    right = np.linalg.inv(truth[6]) @ truth[1]
    poses, summary = pose_choice(tmp_path, truth, odom, right)
    assert summary["poses"] == "optimized", summary
    sc = summary["consistency"]
    assert sc["optimized"]["inliers"] > sc["odometry"]["inliers"], sc
    # The solve lands nearer the truth than odometry. Not exactly on it: with rotation
    # weighted 10x below translation (the info matrices here, like RTAB-Map's), the
    # least-squares optimum bends the chain a few degrees instead of shortening
    # steps, cost 3.3 against 15 for the translation-only answer.
    def err(P):
        return np.mean([np.linalg.norm(P[i][:3, 3] - truth[i][:3, 3]) for i in truth])
    assert err(poses) < 0.5 * err(odom), (err(poses), err(odom))


def test_forced_pose_sources_and_bad_name(tmp_path):
    truth = slide(4)
    links = chain(truth) + [(4, 1, R.GLOBAL_CLOSURE,
                             np.linalg.inv(truth[4]) @ truth[1] @ pose(0.3, 0, 0))]
    make_db(tmp_path / "m.db", truth, links, depth_of=lambda i: wall_depth(truth[i]),
            size=(BW, BH, BF))
    con = R.connect(tmp_path / "m.db")
    try:
        odo, _, s1 = R.graph_poses(con, source="odometry")
        opt, _, s2 = R.graph_poses(con, source="optimized")
        assert (s1["poses"], s2["poses"]) == ("odometry", "optimized")
        assert "consistency" not in s1 and "consistency" not in s2   # nothing measured
        np.testing.assert_allclose(odo[4], truth[4], atol=1e-5)
        assert not np.allclose(opt[4], truth[4], atol=1e-3)
        with pytest.raises(R.DbError):
            R.graph_poses(con, source="best")
    finally:
        con.close()
