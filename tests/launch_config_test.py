"""Offline tests for the Scenarios tab's launch form (webui/session.py).

The form sends SETTINGS (map, mode, robots, ...) to a launcher reachable on
0.0.0.0, so the rules worth pinning are: settings become the same RUN_FLAGS
argv a preset would, a map is chosen by library NAME only (never a path), and
a preset written from the form is validated before it can land in configs/.
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import pytest
import yaml

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "webui"))
sys.path.insert(0, str(REPO / "scripts"))

import maps_lib  # noqa: E402
import session as sm  # noqa: E402


@pytest.fixture
def lib(tmp_path, monkeypatch):
    """Temp maps/ (one complete entry, one grid-only), working waypoints,
    scenario dir and $HOME — nothing touches the repo or ~/.ros."""
    maps = tmp_path / "maps"
    maps.mkdir()
    monkeypatch.setattr(maps_lib, "MAPS_DIR", maps)
    monkeypatch.setattr(maps_lib, "NAV_WAYPOINTS_FILE", tmp_path / "wp.json")
    monkeypatch.setattr(sm, "SCENARIO_DIR", tmp_path / "scenarios")
    (tmp_path / "scenarios").mkdir()
    monkeypatch.setenv("HOME", str(tmp_path / "home"))
    for name, state in (("wh", "complete"), ("gridonly", "pending")):
        maps_lib.main(["init", name])
        if state == "complete":
            (maps / name / "map.db").write_bytes(b"SQLite format 3\x00" + b"\x00" * 64)
            m = json.loads((maps / name / "map.json").read_text())
            m["dbs"] = {"robot_0": {"file": "map.db", "state": "complete"}}
            m["db_state"] = "complete"
            (maps / name / "map.json").write_text(json.dumps(m))
    return tmp_path


def _flags(run, platform="sim"):
    return sm.build_argv(run, platform)


def test_new_map_is_a_fresh_run(lib):
    run = sm.config_to_run({"map": "new", "robots": 2})
    assert not run["resume"] and not run["localize"] and run["map"] is None
    assert "--robots" in _flags(run) and "2" in _flags(run)


@pytest.mark.parametrize("mode,flag,other", [("extend", "--resume", "--localize"),
                                             ("readonly", "--localize", "--resume")])
def test_library_map_modes(lib, mode, flag, other):
    run = sm.config_to_run({"map": "wh", "mode": mode})
    argv = _flags(run)
    assert flag in argv and other not in argv
    assert argv[argv.index("--map") + 1] == str(maps_lib.MAPS_DIR / "wh" / "map.db")


def test_working_map_needs_a_working_db(lib):
    with pytest.raises(sm.ScenarioError, match="no working map"):
        sm.config_to_run({"map": "working"})
    db = Path(lib / "home" / ".ros" / "sortbots_robot_0.db")
    db.parent.mkdir(parents=True)
    db.write_bytes(b"x")
    run = sm.config_to_run({"map": "working", "mode": "readonly"})
    assert run["localize"] and run["map"] is None


@pytest.mark.parametrize("bad", ["../etc", "/home/x/map.db", "wh/../../x", "WH", ""])
def test_map_is_a_library_name_never_a_path(lib, bad):
    with pytest.raises(sm.ScenarioError):
        sm.config_to_run({"map": bad})


def test_grid_only_map_cannot_be_loaded(lib):
    with pytest.raises(sm.ScenarioError, match="pose graph is pending"):
        sm.config_to_run({"map": "gridonly", "mode": "extend"})


@pytest.mark.parametrize("config,match", [
    ({"argv": "rm -rf /"}, "unknown setting"),
    ({"robots": 99}, "--robots"),
    ({"scene": "moon"}, "--scene"),
    ({"mode": "both"}, "mode"),
    ({"waypoints": "all"}, "waypoints"),
])
def test_bad_settings_are_rejected(lib, config, match):
    with pytest.raises(sm.ScenarioError, match=match):
        sm.config_to_run(config)


def test_real_platform_takes_only_its_settings(lib):
    run = sm.config_to_run({"map": "wh", "mode": "readonly"}, "real")
    assert _flags(run, "real")[:2] == ["--robot-id", "robot_0"]
    with pytest.raises(sm.ScenarioError, match="platform 'real'"):
        sm.config_to_run({"robots": 2}, "real")


def test_form_matches_the_equivalent_preset(lib):
    """`custom map=wh mode=extend` must launch exactly what library_resume does."""
    preset = sm._validate(lib / "scenarios" / "p.yaml", {
        "name": "p", "run": {"scene": "nvidia", "robots": 1, "chase_cam": False,
                             "explore": True, "resume": True, "map": "wh"}})
    assert _flags(sm.config_to_run({"map": "wh", "mode": "extend"})) == _flags(preset["run"])


def test_every_shipped_preset_round_trips_through_the_form():
    for sc in sm.load_scenarios():
        assert sc["status"] != "invalid", sc.get("error")
        out = sm.run_to_config(sc["run"], sc["platform"], sc["capture"]["bag"])
        assert out["config"] is not None, (sc["name"], out.get("reason"))


def test_preset_written_from_the_form_is_portable_and_validated(lib):
    sc = sm.write_preset("mine", {"map": "wh", "mode": "extend", "robots": 2}, title="Mine")
    raw = yaml.safe_load((lib / "scenarios" / "mine.yaml").read_text())
    assert raw["origin"] == "dashboard"
    assert raw["run"]["map"] == "wh"  # a name, not /tmp/.../maps/wh/map.db
    assert sc["run"]["robots"] == 2
    # And it launches the same thing it was saved from.
    assert _flags(sc["run"]) == _flags(sm.config_to_run({"map": "wh", "mode": "extend", "robots": 2}))


def test_preset_overwrite_rules(lib):
    sm.write_preset("mine", {"map": "new"})
    with pytest.raises(sm.ScenarioError, match="already exists"):
        sm.write_preset("mine", {"map": "new"})
    sm.write_preset("mine", {"map": "new", "robots": 2}, overwrite=True)
    (lib / "scenarios" / "handmade.yaml").write_text("name: handmade\nrun: {}\n")
    with pytest.raises(sm.ScenarioError, match="hand-written"):
        sm.write_preset("handmade", {"map": "new"}, overwrite=True)
    with pytest.raises(sm.ScenarioError, match="must match"):
        sm.write_preset("../evil", {"map": "new"})


@pytest.mark.parametrize("policy,expected", [("map", ["A"]), ("keep", ["Z"]), ("none", [])])
def test_waypoint_policy(lib, policy, expected):
    maps_lib.write_working_waypoints("sim", [{"name": "A", "x": 1, "y": 2, "yaw": 0}])
    maps_lib.snapshot_waypoints("wh", "sim")
    maps_lib.write_working_waypoints("sim", [{"name": "Z", "x": 0, "y": 0, "yaw": 0}])
    mgr = sm.SessionManager.__new__(sm.SessionManager)
    mgr.platform = "sim"
    mgr._apply_waypoint_policy(sm.config_to_run({"map": "wh", "waypoints": policy}))
    assert [w["name"] for w in maps_lib.read_working_waypoints("sim")] == expected
