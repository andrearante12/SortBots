"""nodes/magnet_pick.py's pure helpers and its config. Offline: no ROS.

    /usr/bin/python3 -m pytest tests/magnet_pick_test.py
"""
from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "nodes"))
sys.path.insert(0, str(REPO_ROOT))

import magnet_pick as mp  # noqa: E402
import pick_planner as pp  # noqa: E402
from simtest import magnet  # noqa: E402

SIZE = [0.24, 0.18, 0.12]
ATTACH = {"cmd": "attach", "package": "pkg_0", "task_id": "t1", "size": SIZE, "face": "+x",
          "at": {"x": -7.9, "y": 9.9}}
DETACH = {"cmd": "detach", "package": "pkg_0", "task_id": "t1", "size": SIZE, "face": "+x",
          "place": {"x": -8.02, "y": 9.9, "z": 0.8, "yaw": 1.5708}}


def test_parse_request_accepts_the_three_commands():
    assert mp.parse_request(json.dumps(ATTACH)) == ATTACH
    assert mp.parse_request(json.dumps(DETACH)) == DETACH
    assert mp.parse_request('{"cmd": "cancel"}') == {"cmd": "cancel"}


@pytest.mark.parametrize("text", [
    "attach", "detach", "", "not json", "[]", '{"cmd": "fly"}', '{"cmd": "attach"}',
    json.dumps({k: v for k, v in ATTACH.items() if k != "at"}),
    json.dumps({k: v for k, v in DETACH.items() if k != "place"}),
])
def test_parse_request_refuses_what_it_cannot_aim_with(text):
    assert mp.parse_request(text) is None


def test_yaw_from_quaternion():
    for yaw in (0.0, 0.5, -2.0, 3.0):
        assert mp.yaw_from_quat(0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)) == \
            pytest.approx(yaw)


def test_oracle_plate_is_the_true_plate_of_the_requested_package():
    pose = {"x": -7.9, "y": 9.9, "z": 0.8, "yaw": 1.5708, "held_by": None}
    (p,) = mp.oracle_plates({"packages": {"pkg_0": pose, "pkg_1": dict(pose, x=0.0)}}, ATTACH)
    centre, normal = magnet.magnet_face_pose(pose, SIZE, "+x")
    assert (p["x"], p["y"], p["z"]) == pytest.approx(centre)
    assert (p["nx"], p["ny"], p["nz"]) == pytest.approx(normal)
    assert p["conf"] == 1.0 and p["age_s"] == 0.0


def test_oracle_sees_nothing_when_held_unrequested_or_placing():
    pose = {"x": -7.9, "y": 9.9, "z": 0.8, "yaw": 1.5708}
    held = {"packages": {"pkg_0": dict(pose, held_by="robot_0")}}
    free = {"packages": {"pkg_0": dict(pose, held_by=None)}}
    assert mp.oracle_plates(held, ATTACH) == []
    assert mp.oracle_plates(free, None) == []
    assert mp.oracle_plates(free, DETACH) == []                 # a place aims at a spot, not a plate
    assert mp.oracle_plates({"packages": {}}, ATTACH) == []
    assert mp.oracle_plates({}, ATTACH) == []


def test_camera_plates_age_with_the_message_and_expire():
    msg = {"stamp": 100.0, "plates": [{"x": 1, "y": 2, "z": 3, "nx": 0, "ny": 1, "nz": 0,
                                       "conf": 0.8, "age_s": 0.2}]}
    (p,) = mp.camera_plates(msg, 100.0)
    assert p["age_s"] == pytest.approx(0.2)
    (p,) = mp.camera_plates(msg, 100.5)
    assert p["age_s"] == pytest.approx(0.7)                     # half a second behind, and older for it
    assert mp.camera_plates(msg, 101.5) == []                   # the detector went quiet
    assert mp.camera_plates(None, 100.0) == []
    assert msg["plates"][0]["age_s"] == 0.2                     # the message itself is not mutated


def test_config_keys_all_exist_in_the_planner_and_agree_with_its_defaults():
    cfg = mp.load_config(mp.DEFAULT_CONFIG)
    assert cfg.pop("stream_hz") > 0
    for key, value in cfg.items():
        assert key in pp.DEFAULTS, f"configs/magnet_pick.yaml: {key} is not a planner setting"
        if isinstance(value, dict):
            assert set(value) == set(pp.DEFAULTS[key]), key
    # one set of numbers in two places: the YAML documents the defaults, it does not fork them
    assert pp.merged(cfg) == pp.merged({})


def test_missing_config_is_an_empty_one_not_a_crash(tmp_path):
    assert mp.load_config(tmp_path / "nope.yaml") == {}
    bad = tmp_path / "bad.yaml"
    bad.write_text("a: [unclosed")
    assert mp.load_config(bad) == {}


def test_the_arm_joint_order_matches_the_simulator():
    # spawn_warehouse.ARM_JOINT_NAMES: the command is dropped if one is missing
    src = (REPO_ROOT / "scripts" / "spawn_warehouse.py").read_text()
    line = next(l for l in src.splitlines() if l.startswith("ARM_JOINT_NAMES"))
    assert json.loads(line.split("=", 1)[1].strip().replace("'", '"')) == mp.ARM_JOINT_NAMES


def test_compose2d_places_a_child_pose_in_its_parent():
    x, y, yaw = mp.compose2d((1.0, 2.0, math.pi / 2), (0.5, 0.0, 0.25))
    assert (x, y, yaw) == pytest.approx((1.0, 2.5, math.pi / 2 + 0.25))
    assert mp.compose2d((0.0, 0.0, 0.0), (3.0, 4.0, 1.0)) == pytest.approx((3.0, 4.0, 1.0))
