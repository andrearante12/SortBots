"""simtest.world + simtest.geometry: schema, kinematics, footprints. Offline.

    /usr/bin/python3 -m pytest tests/simtest_world_test.py
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT))

from simtest import geometry, world  # noqa: E402


def _walker(**path):
    p = {"waypoints": [[0.0, 0.0], [4.0, 0.0]], "speed_mps": 1.0, "mode": "pingpong"}
    p.update(path)
    return {"name": "t", "actors": [{"id": "w", "shape": "cylinder", "radius": 0.25,
                                     "height": 1.7, "path": p}]}


# -- every committed world must load ------------------------------------------

@pytest.mark.parametrize("name", sorted(p.stem for p in world.WORLDS_DIR.glob("*.yaml")))
def test_committed_worlds_validate(name):
    w = world.load(name)
    assert w["name"] == name
    ids = [i["id"] for i in w["props"] + w["actors"]]
    assert len(ids) == len(set(ids))


def test_committed_worlds_actors_clear_of_props():
    """No committed actor walks through one of its own world's props — a
    layout bug that would read as 'robot fine, actor clipping a crate'."""
    for p in world.WORLDS_DIR.glob("*.yaml"):
        w = world.load(p.stem)
        for a in w["actors"]:
            for t in [i * 0.25 for i in range(int(world.cycle_s(a["path"]) * 4) + 1)]:
                fa = world.actor_footprint(a, a["path"]["start_at_s"] + t)
                for prop in w["props"]:
                    assert geometry.distance(fa, world.prop_footprint(prop)) > 0.05, \
                        f"{p.stem}: {a['id']} touches {prop['id']} at t={t}"


# -- validation ----------------------------------------------------------------

@pytest.mark.parametrize("mutate,err", [
    (lambda w: w.update(name="Bad Name"), "name"),
    (lambda w: w.update(bogus=1), "unknown key"),
    (lambda w: w["actors"][0].update(shape="usd"), "shape"),
    (lambda w: w["actors"][0].update(radius=0), "radius"),
    (lambda w: w["actors"][0]["path"].update(waypoints=[[0, 0]]), "two"),
    (lambda w: w["actors"][0]["path"].update(waypoints=[[0, 0], [0, 0]]), "repeats"),
    (lambda w: w["actors"][0]["path"].update(speed_mps=9), "speed_mps"),
    (lambda w: w["actors"][0]["path"].update(mode="teleport"), "mode"),
    (lambda w: w["actors"][0]["path"].update(waypoints=[[0, 0], [1, 0], [0, 0]], mode="loop"),
     "loop closes itself"),
    (lambda w: w["actors"][0].update(id="w0", path=None), "path"),
    (lambda w: w.update(props=[{"id": "w", "shape": "box", "size": [1, 1, 1],
                                "pose": [0, 0]}]), "duplicate"),
    (lambda w: w.update(props=[{"id": "p", "shape": "usd", "ref": "/etc/passwd.usd",
                                "pose": [0, 0], "footprint": [1, 1]}]), "ref"),
    (lambda w: w.update(props=[{"id": "p", "shape": "usd",
                                "ref": "Isaac/../../x.usd", "pose": [0, 0],
                                "footprint": [1, 1]}]), "ref"),
    (lambda w: w.update(props=[{"id": "p", "shape": "usd", "ref": "Isaac/Props/x.usd",
                                "pose": [0, 0]}]), "footprint"),
    (lambda w: w.update(scenes=["mars"]), "scenes"),
])
def test_validation_rejects(mutate, err):
    raw = _walker()
    mutate(raw)
    with pytest.raises(world.WorldError, match=err):
        world.validate(raw)


def test_stem_must_match():
    with pytest.raises(world.WorldError, match="stem"):
        world.validate(_walker(), stem="other")


def test_path_for_rejects_traversal():
    for bad in ("../x", "a/b", "x.yaml", ""):
        with pytest.raises(world.WorldError):
            world.path_for(bad)


# -- kinematics ----------------------------------------------------------------

def _path(**kw):
    return world.validate(_walker(**kw))["actors"][0]["path"]


def test_pingpong_positions_and_heading():
    p = _path()
    assert world.cycle_s(p) == pytest.approx(8.0)
    s = world.pose_at(p, 1.0)
    assert (s["x"], s["y"], s["moving"]) == (pytest.approx(1.0), 0.0, True)
    assert s["yaw"] == pytest.approx(0.0) and s["vx"] == pytest.approx(1.0)
    back = world.pose_at(p, 5.0)
    assert back["x"] == pytest.approx(3.0)
    assert back["yaw"] == pytest.approx(math.pi)
    assert world.pose_at(p, 9.0)["x"] == pytest.approx(1.0)  # periodic


def test_waits_before_start_and_dwells():
    p = _path(start_at_s=2.0, dwell_s=1.0)
    before = world.pose_at(p, 1.0)
    assert (before["x"], before["moving"]) == (0.0, False)
    assert world.pose_at(p, 2.5)["moving"] is False      # initial dwell
    assert world.pose_at(p, 4.0)["x"] == pytest.approx(1.0)
    assert world.cycle_s(p) == pytest.approx(10.0)


def test_once_holds_the_end():
    p = _path(mode="once")
    end = world.pose_at(p, 100.0)
    assert (end["x"], end["moving"]) == (4.0, False)


def test_loop_closes():
    p = _path(waypoints=[[0, 0], [2, 0], [2, 2]], mode="loop")
    period = world.cycle_s(p)
    assert period == pytest.approx(4.0 + math.hypot(2, 2))
    end = world.pose_at(p, period - 1e-6)
    assert math.hypot(end["x"], end["y"]) < 1e-3


def test_swept_samples_cover_path():
    a = world.validate(_walker())["actors"][0]
    pts = world.swept_samples(a, 0.5)
    xs = [x for x, _ in pts]
    assert min(xs) == 0.0 and max(xs) == 4.0


# -- geometry ------------------------------------------------------------------

def test_distances():
    box = geometry.rect(0, 0, 0, 2, 2)
    assert geometry.distance(box, geometry.circle(3, 0, 1)) == pytest.approx(1.0)
    assert geometry.distance(box, geometry.circle(0.5, 0, 0.1)) == 0.0
    assert geometry.distance(geometry.circle(0, 0, 1), geometry.circle(3, 0, 1)) == \
        pytest.approx(1.0)
    rot = geometry.rect(4, 0, math.pi / 4, 2, 2)  # corner points at the box
    assert geometry.distance(box, rot) == pytest.approx(4 - 1 - math.sqrt(2))
    assert geometry.distance(box, geometry.rect(1.5, 0, 0, 2, 2)) == 0.0


def test_footprint_poly_is_ccw_and_rotated():
    pts = ((0.25, 0.2), (0.25, -0.2), (-0.25, -0.2), (-0.25, 0.2))  # Nav2's CW order
    fp = geometry.footprint_poly(pts, 1.0, 1.0, math.pi / 2)
    assert geometry.contains(fp, (1.0, 1.24))       # forward is now +y
    assert not geometry.contains(fp, (1.24, 1.0))


def test_json_round_trip():
    import json
    for s in (geometry.circle(1, 2, 0.3), geometry.rect(1, 2, 0.5, 1, 2)):
        assert geometry.from_json(json.loads(json.dumps(s))) == s
