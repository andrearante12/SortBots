"""splat/dataset.py: rtabmap-export output -> keyframe set. numpy only."""
import json
import sys
from pathlib import Path

import numpy as np
import pytest

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "splat"))
sys.path.insert(0, str(Path(__file__).resolve().parent))

import dataset  # noqa: E402
from splat_helpers import CALIB, OPTICAL_Q, make_export, write_pcl_ply  # noqa: E402


def test_parse_camera_poses_is_opencv_camera_to_world():
    qx, qy, qz, qw = OPTICAL_Q
    text = f"#timestamp x y z qx qy qz qw id\n3.5 1 2 1.19 {qx} {qy} {qz} {qw} 7\n"
    poses = dataset.parse_camera_poses(text)
    stamp, T = poses[7]
    assert stamp == 3.5
    np.testing.assert_allclose(T[:3, 3], [1, 2, 1.19])
    R = T[:3, :3]
    # The optical frame RTAB-Map exports: camera z (forward) is the robot's
    # +x, camera x (right) is -y, camera y (down) is -z. If this ever flips,
    # every splat trains against mirrored cameras.
    np.testing.assert_allclose(R @ [0, 0, 1], [1, 0, 0], atol=1e-9)
    np.testing.assert_allclose(R @ [1, 0, 0], [0, -1, 0], atol=1e-9)
    np.testing.assert_allclose(R @ [0, 1, 0], [0, 0, -1], atol=1e-9)


def test_parse_camera_poses_rejects_short_rows():
    with pytest.raises(dataset.DatasetError):
        dataset.parse_camera_poses("1 2 3 4 5 6 7 8\n")


def test_parse_calib_reads_opencv_yaml():
    intr = dataset.parse_calib(CALIB.format(id=3))
    assert intr == {"w": 848, "h": 480, "fl_x": pytest.approx(599.765435545),
                    "fl_y": pytest.approx(599.765435545), "cx": 424.0, "cy": 240.0}


def test_parse_calib_handles_opencv_matrix_tags():
    text = CALIB.format(id=1).replace("camera_matrix:\n", "camera_matrix: !!opencv-matrix\n")
    assert dataset.parse_calib(text)["fl_x"] == pytest.approx(599.765435545)


def test_points_ply_round_trip(tmp_path):
    xyz = np.array([[0, 1, 2], [3, 4, 5]], np.float32)
    rgb = np.array([[1, 2, 3], [4, 5, 6]], np.uint8)
    dataset.write_points_ply(tmp_path / "p.ply", xyz, rgb)
    v = dataset.read_ply_vertices(tmp_path / "p.ply")
    np.testing.assert_array_equal(np.stack([v["x"], v["y"], v["z"]], 1), xyz)
    np.testing.assert_array_equal(np.stack([v["red"], v["green"], v["blue"]], 1), rgb)


def test_reads_pcl_ply_and_ignores_trailing_camera_element(tmp_path):
    xyz = np.array([[1, 2, 3]], np.float32)
    write_pcl_ply(tmp_path / "c.ply", xyz, np.array([[9, 8, 7]], np.uint8))
    v = dataset.read_ply_vertices(tmp_path / "c.ply")
    assert len(v) == 1 and v["red"][0] == 9 and v["z"][0] == 3


def test_spawn_anchor_uses_the_bringup_roster():
    T = dataset.spawn_anchor("nvidia", "robot_0")
    np.testing.assert_allclose(T[:3, 3], [-7.15, 11.62, 0.05])
    np.testing.assert_allclose(T[:3, :3], np.eye(3))
    np.testing.assert_allclose(dataset.spawn_anchor("real", "robot_0")[:3, 3], [0, 0, 0])
    with pytest.raises(dataset.DatasetError):
        dataset.spawn_anchor("real", "robot_1")   # no real hardware for robot_1


def test_build_dataset_merges_robots_into_map_frame(tmp_path):
    exp = tmp_path / "export"
    make_export(exp, "robot_0", {1: (0, 0, 1.2), 2: (1, 0, 1.2), 3: (2, 0, 1.2)},
                missing_rgb=(2,))
    make_export(exp, "robot_1", {5: (0, 0, 1.2)},
                cloud=np.array([[0, 0, 0]], np.float32))
    a0 = np.eye(4)
    a0[:3, 3] = (10, 20, 0)
    a1 = np.eye(4)
    a1[:3, 3] = (-5, 0, 0)
    out = tmp_path / "ds"
    t = dataset.build_dataset(
        [{"export_dir": exp, "prefix": "robot_0", "robot_id": "robot_0", "anchor": a0},
         {"export_dir": exp, "prefix": "robot_1", "robot_id": "robot_1", "anchor": a1}],
        out, meta={"map": "m"})

    # Node 2 has no image: dropped, not fatal.
    assert [(f["robot_id"], f["id"]) for f in t["frames"]] == \
        [("robot_0", 1), ("robot_0", 3), ("robot_1", 5)]
    on_disk = json.loads((out / "transforms.json").read_text())
    assert on_disk == t
    assert t["frame_convention"] == "opencv" and t["world_frame"] == "map"
    centres = [np.array(f["transform_matrix"])[:3, 3] for f in t["frames"]]
    np.testing.assert_allclose(centres, [[10, 20, 1.2], [12, 20, 1.2], [-5, 0, 1.2]])
    for f in t["frames"]:
        assert (out / f["file_path"]).is_file()
        assert (out / f["depth_file_path"]).is_file()
        assert f["w"] == 848 and f["h"] == 480
    # Init cloud carries both robots' points, each shifted by its own anchor.
    v = dataset.read_ply_vertices(out / "points3d.ply")
    pts = {tuple(np.round(p, 3)) for p in np.stack([v["x"], v["y"], v["z"]], 1)}
    assert (11.0, 20.0, 0.0) in pts and (-5.0, 0.0, 0.0) in pts
    assert t["init_points"] == 4


def test_build_dataset_budgets_the_init_cloud(tmp_path):
    exp = tmp_path / "export"
    rng = np.random.default_rng(0)
    make_export(exp, "robot_0", {1: (0, 0, 1)}, cloud=rng.uniform(0, 10, (5000, 3)))
    t = dataset.build_dataset(
        [{"export_dir": exp, "prefix": "robot_0", "robot_id": "robot_0", "anchor": np.eye(4)}],
        tmp_path / "ds", init_points=500, init_voxel=0.01)
    assert 0 < t["init_points"] <= 500


def test_build_dataset_stride_and_empty_export(tmp_path):
    exp = tmp_path / "export"
    make_export(exp, "robot_0", {i: (i, 0, 1) for i in range(1, 7)})
    src = [{"export_dir": exp, "prefix": "robot_0", "robot_id": "robot_0", "anchor": np.eye(4)}]
    t = dataset.build_dataset(src, tmp_path / "ds", stride=3)
    assert [f["id"] for f in t["frames"]] == [1, 4]
    (exp / "robot_0_camera_poses.txt").write_text("#timestamp x y z qx qy qz qw id\n")
    with pytest.raises(dataset.DatasetError):
        dataset.build_dataset(src, tmp_path / "ds2")
