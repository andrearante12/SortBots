"""scripts/calibration.sh and the calibration kept in configs/calibration/: every file the arm
tools need is in the repo and parses, install puts them where the tools look and keeps a
differing old file, save brings a recalibration back. Runs against a temp HF_LEROBOT_CALIBRATION:
never this machine's real one."""
import json
import os
import subprocess
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
SCRIPT = REPO / "scripts" / "calibration.sh"
SNAPSHOT = REPO / "configs" / "calibration" / "robots"


def _run(cmd, home):
    env = {**os.environ, "HF_LEROBOT_CALIBRATION": str(home)}
    r = subprocess.run(["bash", str(SCRIPT), cmd], env=env, capture_output=True, text=True, timeout=30)
    assert r.returncode == 0, r.stderr
    return r.stdout


def _listed():
    files = []
    in_list = False
    for line in SCRIPT.read_text().splitlines():
        if line.startswith("FILES=("):
            in_list = True
        elif in_list and line.startswith(")"):
            break
        elif in_list and line.strip():
            files.append(line.split()[0])
    return files


def test_every_listed_file_is_in_the_repo_and_parses():
    files = _listed()
    assert len(files) == 9
    for f in files:
        data = json.loads((SNAPSHOT / f).read_text())
        assert data, f
    arm = json.loads((SNAPSHOT / "so_follower/sortbots_arm.json").read_text())
    assert set(arm) == {"shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"}
    head = json.loads((SNAPSHOT / "sortbots_head/sortbots_head.json").read_text())
    assert set(head) == {"head_pan", "head_tilt"}


def test_install_on_a_fresh_machine_puts_every_file_where_the_tools_look(tmp_path):
    out = _run("install", tmp_path)
    for f in _listed():
        assert (tmp_path / "robots" / f).read_bytes() == (SNAPSHOT / f).read_bytes()
    assert out.count("installed") == 9
    assert "already up to date" in _run("install", tmp_path)
    assert _run("status", tmp_path).count("same") == 9


def test_install_keeps_a_differing_old_file(tmp_path):
    old = tmp_path / "robots" / "so_follower" / "sortbots_arm.json"
    old.parent.mkdir(parents=True)
    old.write_text('{"an": "older calibration"}')
    out = _run("install", tmp_path)
    assert "replaced" in out
    kept = list(old.parent.glob("sortbots_arm.before-install-*.json"))
    assert len(kept) == 1 and json.loads(kept[0].read_text()) == {"an": "older calibration"}
    assert old.read_bytes() == (SNAPSHOT / "so_follower/sortbots_arm.json").read_bytes()


def test_status_names_what_differs_and_what_is_missing(tmp_path):
    _run("install", tmp_path)
    (tmp_path / "robots" / "sortbots_head" / "pick_rest_pose.json").write_text("{}")
    (tmp_path / "robots" / "sortbots_head" / "pick_place_pose.json").unlink()
    st = _run("status", tmp_path)
    assert "differs       sortbots_head/pick_rest_pose.json" in st
    assert "missing here  sortbots_head/pick_place_pose.json" in st
