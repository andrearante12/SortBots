"""Offline tests for webui/session.py's sim/real platform split (no ROS).

The Scenarios tab is a remote process launcher on 0.0.0.0, so the rules worth
pinning are the containment ones: a real-robot scenario may only use
run_robot.sh's small flag set, and a console never starts another platform's
scenario.
"""
from __future__ import annotations

import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "webui"))

import session as sm  # noqa: E402


def _validate(tmp_path, raw):
    path = tmp_path / f"{raw['name']}.yaml"
    return sm._validate(path, raw)


def test_platform_defaults_to_sim(tmp_path):
    s = _validate(tmp_path, {"name": "x", "run": {"scene": "primitive"}})
    assert s["platform"] == "sim"
    assert sm.launcher_argv(s, s["run"])[0].endswith("scripts/run_demo.sh")


def test_real_scenario_uses_run_robot_and_only_its_flags(tmp_path):
    s = _validate(tmp_path, {"name": "r", "platform": "real",
                             "run": {"robot_id": "robot_0", "resume": True}})
    argv = sm.launcher_argv(s, s["run"])
    assert argv[0].endswith("scripts/run_robot.sh")
    assert argv[1:] == ["--robot-id", "robot_0", "--resume", "--slam-role", "all",
                        "--dynamic-seg", "model", "--nav",
                        "--keep-console"]   # movers masked and nav + tasks on by default
    # Defaults come from the real table: no sim keys leak into `run`.
    assert set(s["run"]) == set(sm.REAL_RUN_FLAGS)


def test_real_slam_role_is_validated(tmp_path):
    s = _validate(tmp_path, {"name": "r", "platform": "real", "run": {"slam_role": "robot"}})
    assert "--slam-role" in sm.launcher_argv(s, s["run"])
    with pytest.raises(sm.ScenarioError, match="slam-role"):
        _validate(tmp_path, {"name": "r", "platform": "real", "run": {"slam_role": "cloud"}})


@pytest.mark.parametrize("key,value", [("scene", "nvidia"), ("robots", 1),
                                       ("headless", True), ("explore", True)])
def test_real_scenario_rejects_sim_only_keys(tmp_path, key, value):
    with pytest.raises(sm.ScenarioError, match="platform 'real'"):
        _validate(tmp_path, {"name": "r", "platform": "real", "run": {key: value}})


def test_real_scenario_rejects_sim_only_override(tmp_path):
    with pytest.raises(sm.ScenarioError, match="overrides"):
        _validate(tmp_path, {"name": "r", "platform": "real", "overrides": ["headless"]})


def test_unknown_platform_rejected(tmp_path):
    with pytest.raises(sm.ScenarioError, match="platform"):
        _validate(tmp_path, {"name": "x", "platform": "mars"})


def test_every_committed_scenario_names_a_valid_platform():
    for s in sm.load_scenarios():
        assert s["status"] != "invalid", s.get("error")
        assert s["platform"] in sm.PLATFORMS


def test_console_refuses_other_platforms_scenario(tmp_path, monkeypatch):
    monkeypatch.setattr(sm, "SESSIONS_DIR", tmp_path)
    monkeypatch.setattr(sm, "CURRENT_POINTER", tmp_path / "current.json")
    sim_console = sm.SessionManager("sim")
    with pytest.raises(sm.ScenarioError, match="platform 'real'"):
        sim_console.start("real_map_fresh")
    real_console = sm.SessionManager("real")
    with pytest.raises(sm.ScenarioError, match="platform 'sim'"):
        real_console.start("explore_fresh")
    # Refused before anything was spawned or written.
    assert not any(tmp_path.iterdir())


def test_detect_platform_env_override(monkeypatch):
    monkeypatch.setenv("SORTBOTS_PLATFORM", "real")
    assert sm.detect_platform() == "real"
    monkeypatch.setenv("SORTBOTS_PLATFORM", "bogus")
    with pytest.raises(ValueError):
        sm.detect_platform()


def test_dynamic_seg_and_policy_flags():
    assert sm.build_argv({"dynamic_seg": "model", "seg_policy": "strict"}, "sim") == [
        "--dynamic-seg", "model", "--seg-policy", "strict"]
    assert sm.build_argv({"dynamic_seg": "off", "seg_policy": "open"}, "sim") == []
    assert sm.build_argv({"dynamic_seg": "model"}, "real") == ["--dynamic-seg", "model"]
    with pytest.raises(sm.ScenarioError, match="expected one of"):
        sm.build_argv({"dynamic_seg": "oracle"}, "real")   # no simulator on hardware
    with pytest.raises(sm.ScenarioError, match="expected one of"):
        sm.build_argv({"seg_policy": "lenient"}, "sim")


def test_recon_source_flag():
    assert sm.build_argv({"recon_source": "octomap"}, "sim") == ["--recon-source", "octomap"]
    assert sm.build_argv({"recon_source": "cloud_map"}, "real") == []
    with pytest.raises(sm.ScenarioError, match="expected one of"):
        sm.build_argv({"recon_source": "splat"}, "sim")


def test_real_nav_flag_reaches_run_robot():
    # Nav2 + the explorer + task_manager on the robot (run_robot.sh --nav): on by default since
    # 2026-11-20 (a queued task waited forever without it); nav: false turns it off.
    import session as sm
    assert "--nav" in sm.build_argv(dict(sm.REAL_RUN_DEFAULTS), "real")
    assert "--nav" in sm.build_argv(sm.config_to_run({"map": "new"}, "real"), "real")   # the launch form's runs too
    assert "--nav" not in sm.build_argv({**sm.REAL_RUN_DEFAULTS, "nav": False}, "real")


def test_real_runs_mask_movers_by_default():
    # dynamic_seg defaults on for the robot: the launch form's runs and presets alike
    import session as sm
    argv = sm.build_argv(dict(sm.REAL_RUN_DEFAULTS), "real")
    assert argv[argv.index("--dynamic-seg") + 1] == "model"
    assert "--dynamic-seg" in sm.build_argv(sm.config_to_run({"map": "new"}, "real"), "real")
    assert "--dynamic-seg" not in sm.build_argv({**sm.REAL_RUN_DEFAULTS, "dynamic_seg": "off"}, "real")
    assert sm.RUN_DEFAULTS["dynamic_seg"] == "off"   # the sim keeps it opt-in
