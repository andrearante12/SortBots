"""scripts/recon/rtabmap_to_colmap.py: RTAB-Map export -> COLMAP, no ROS."""
import importlib.util
import math
from pathlib import Path

import numpy as np

spec = importlib.util.spec_from_file_location(
    "r2c", Path(__file__).resolve().parent.parent / "scripts" / "recon" / "rtabmap_to_colmap.py")
C = importlib.util.module_from_spec(spec)
spec.loader.exec_module(C)

K = (500.0, 500.0, 320.0, 240.0)
# base_link -> optical, camera 0.4 m up, as RTAB-Map writes local_transform
LOCAL = "0., 0., 1., 0., -1., 0., 0., 0., 0., -1., 0., 0.4"
CALIB = f"""%YAML:1.0
---
camera_name: "x"
image_width: 640
image_height: 480
camera_matrix:
   rows: 3
   cols: 3
   data: [ {K[0]}, 0., {K[2]}, 0.,
       {K[1]}, {K[3]}, 0., 0., 1. ]
local_transform:
   rows: 3
   cols: 4
   data: [ {LOCAL} ]
"""


def write_export(d: Path, poses, points):
    (d / "s_rgb").mkdir()
    (d / "s_calib").mkdir()
    lines = ["#timestamp x y z qx qy qz qw"]
    for stamp, (xyz, yaw) in poses.items():
        lines.append(f"{stamp} {xyz[0]} {xyz[1]} {xyz[2]} 0 0 {math.sin(yaw / 2)} {math.cos(yaw / 2)}")
        (d / "s_rgb" / f"{stamp}.jpg").write_bytes(b"jpg")
        (d / "s_calib" / f"{stamp}.yaml").write_text(CALIB)
    (d / "s_poses.txt").write_text("\n".join(lines) + "\n")
    v = np.zeros(len(points), dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
                                     ("red", "u1"), ("green", "u1"), ("blue", "u1")])
    v["x"], v["y"], v["z"] = points.T
    v["red"] = 200
    hdr = (f"ply\nformat binary_little_endian 1.0\nelement vertex {len(v)}\n"
           "property float x\nproperty float y\nproperty float z\n"
           "property uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n")
    (d / "s_cloud.ply").write_bytes(hdr.encode() + v.tobytes())


def project(row, p):
    qw, qx, qy, qz, tx, ty, tz = map(float, row[1:8])
    pc = C.quat_to_R(qx, qy, qz, qw) @ p + [tx, ty, tz]
    return K[0] * pc[0] / pc[2] + K[2], K[1] * pc[1] / pc[2] + K[3], pc[2]


def test_point_ahead_of_robot_projects_to_image_center(tmp_path):
    # robot at (1, 2) facing +y (yaw 90 deg); a point 3 m ahead at camera height
    write_export(tmp_path, {"100.000000": ((1.0, 2.0, 0.0), math.pi / 2)},
                 np.array([[1.0, 5.0, 0.4]]))
    r = C.convert(tmp_path / "s", tmp_path / "out")
    assert r == {"images": 1, "cameras": 1, "points": 1}
    rows = [l.split() for l in (tmp_path / "out/sparse/0/images.txt").read_text().splitlines()
            if l and not l.startswith("#")]
    u, v, z = project(rows[0], np.array([1.0, 5.0, 0.4]))
    assert abs(u - K[2]) < 1e-6 and abs(v - K[3]) < 1e-6 and abs(z - 3.0) < 1e-6
    # and a point 1 m to the robot's LEFT lands left of center
    u, _, _ = project(rows[0], np.array([0.0, 5.0, 0.4]))
    assert u < K[2]
    pts = (tmp_path / "out/sparse/0/points3D.txt").read_text().splitlines()[1].split()
    assert pts[4:7] == ["200", "0", "0"]


def test_quat_roundtrip():
    rng = np.random.default_rng(1)
    for _ in range(50):
        q = rng.normal(size=4)
        q /= np.linalg.norm(q)
        R = C.quat_to_R(*q)
        w, x, y, z = C.R_to_quat(R)
        assert np.allclose(C.quat_to_R(x, y, z, w), R, atol=1e-9)


def test_images_without_pose_are_skipped(tmp_path):
    write_export(tmp_path, {"1.000000": ((0, 0, 0), 0.0)}, np.zeros((3, 3)) + [2, 0, 0.4])
    (tmp_path / "s_rgb" / "2.000000.jpg").write_bytes(b"x")  # no pose for this one
    assert C.convert(tmp_path / "s", tmp_path / "o")["images"] == 1
