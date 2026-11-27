"""fleetsim/preview.py: the scenario editor's floor is the floor a run gets."""
from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from fleetsim import config, preview, sim  # noqa: E402
from fleetsim import grid as G  # noqa: E402

SCENARIOS = sorted(p.stem for p in config.SCENARIOS_DIR.glob("*.yaml"))


def same_world(scn):
    real = sim.Sim(scn)
    real._world()
    recorded = next(e for e in real.events if e["s"] == "world")
    s, homes, err = preview.build(scn)
    w = preview.world_json(scn, s, homes)
    assert err is None
    assert np.array_equal(s.static, real.static)
    assert w["grid"] == recorded["grid"]
    assert w["waypoints"] == recorded["waypoints"]
    assert w["homes"] == [recorded["homes"][r.rid] for r in real.robots]
    assert w["gateway"] == recorded["gateway"]
    assert w["notes"] == real.notes
    return w


@pytest.mark.parametrize("name", SCENARIOS)
def test_every_scenarios_preview_is_its_runs_world(name):
    same_world(config.load(name))


@pytest.mark.parametrize("synthetic", [{"width_m": 30, "height_m": 22, "docks": 4, "cross_every_m": 12},
                                       {"aisle_m": 3.0, "rack_depth_m": 2.0, "staging_m": 5.0}])
def test_synthetic_options_reach_the_preview(synthetic):
    scn = config.resolve({"name": "t", "robots": 4, "config": {"world": {"synthetic": synthetic}}})
    same_world(scn)


def test_a_saved_map_without_waypoints_gets_the_runs_random_spots(tmp_path, monkeypatch):
    d = tmp_path / "box"
    d.mkdir()
    w, h = 240, 200  # 12 x 10 m: room for six spots 2 m apart
    px = bytearray([254]) * (w * h)
    for x in range(w):
        px[x] = px[(h - 1) * w + x] = 0
    for y in range(h):
        px[y * w] = px[y * w + w - 1] = 0
    (d / "grid.pgm").write_bytes(f"P5\n{w} {h}\n255\n".encode() + bytes(px))
    (d / "grid.yaml").write_text("image: grid.pgm\nresolution: 0.05\norigin: [2.0, -1.0, 0.0]\nnegate: 0\n")
    monkeypatch.setattr(G, "MAPS_DIR", tmp_path)
    scn = config.resolve({"name": "t", "robots": 2, "seed": 3,
                          "config": {"world": {"source": "map", "map": "box", "random_waypoints": 6}}})
    world = same_world(scn)
    assert sorted(world["waypoints"]) == [f"p_{i}" for i in range(6)]
    assert world["origin"] == [2.0, -1.0]


def test_check_reports_problems_and_never_raises():
    out = preview.check({"name": "t", "robots": 2,
                         "tasks": [{"at_s": 1, "pickup": "shelf_99_0", "dropoff": "dock_0"}]})
    assert [p["where"] for p in out["problems"]] == ["tasks[0].pickup"]
    # a robot the fleet lacks is config.resolve's to refuse, as an error
    out = preview.check({"name": "t", "robots": 2, "disturbances": [{"at_s": 1, "revive": "robot_5"}]})
    assert "disturbances[0].revive" in out["error"]
    assert preview.check({"name": "Bad"}) == {"error": f"name: must match {config.NAME_RX.pattern}"}
    json.dumps(preview.check({"name": "t"}))  # all of it is JSON
