"""arm_limits: the zero shift that keeps stored angles meaning the same physical pose."""
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
import arm_limits as L  # noqa: E402


def _cal(lo, hi):
    return {"id": 1, "drive_mode": 0, "homing_offset": 0, "range_min": lo, "range_max": hi}


def test_plan_widens_only_and_reports_the_zero_shift(tmp_path, monkeypatch):
    cal = {j: _cal(1000, 3000) for j in L.ARM}
    (tmp_path / "cal.json").write_text(json.dumps(cal))
    stops = {j: [1000, 3000] for j in L.ARM}
    stops["wrist_flex"] = [600, 3100]       # 400 raw further one way, 100 the other
    stops["elbow_flex"] = [1100, 2900]      # inside the limits: never narrowed
    (tmp_path / "stops.json").write_text(json.dumps({"raw": stops}))
    monkeypatch.setattr(L, "ARM_CAL", tmp_path / "cal.json")
    monkeypatch.setattr(L, "STOPS", tmp_path / "stops.json")
    p = L.plan()
    m = round(L.MARGIN_DEG * L.MAX_RES / 360)
    assert set(p) == {"wrist_flex"}
    lo0, hi0, lo1, hi1, shift = p["wrist_flex"]
    assert (lo1, hi1) == (600 + m, 3100 - m)
    assert abs(shift - ((lo1 + hi1) / 2 - 2000) * 360 / L.MAX_RES) < 1e-9


def test_a_stored_pose_keeps_its_raw_position_after_the_shift():
    old_mid, new_lo, new_hi = 2000, 640, 3070
    new_mid = (new_lo + new_hi) / 2
    raw = 2900
    old_deg = L.to_deg(raw, old_mid)
    shift = (new_mid - old_mid) * 360 / L.MAX_RES
    pose = {"arm_deg": [0.0, 0.0, 0.0, old_deg, 0.0]}
    L.shift_poses(pose, {"wrist_flex": shift})
    assert abs(pose["arm_deg"][3] - L.to_deg(raw, new_mid)) < 1e-3   # same servo position
    poses = [{"arm_deg": [1, 2, 3, 4, 5]}, {"other": 1}]
    assert L.shift_poses(poses, {"shoulder_pan": 1.5}) == 1 and poses[0]["arm_deg"][0] == -0.5
