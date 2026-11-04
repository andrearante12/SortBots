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
    ids = [i["id"] for i in w["props"] + w["actors"] + w["packages"]]
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


# -- packages ------------------------------------------------------------------

def _deck_world(**pkg):
    p = {"id": "pkg", "pose": [0, 0, 0], "z": 0.8, "magnet_face": "+x"}
    p.update(pkg)
    return {"name": "t",
            "props": [{"id": "deck", "shape": "box", "size": [0.5, 0.5, 0.8], "pose": [0, 0, 0]}],
            "packages": [p]}


def test_package_defaults_and_support():
    w = world.validate(_deck_world())
    pkg = w["packages"][0]
    assert pkg["size"] == world.DEFAULT_PACKAGE_SIZE and pkg["magnet_face"] == "+x"
    assert world.support_for(w, pkg)["id"] == "deck"
    floor = world.validate({"name": "t", "packages": [{"id": "p", "pose": [3, 3]}]})
    assert world.support_for(floor, floor["packages"][0]) is None


def test_package_may_overhang_its_deck_a_little():
    # 0.24 x 0.18 box, 2 cm past the deck's east edge: fine; 8 cm: it would fall
    world.validate(_deck_world(pose=[0.15, 0, 0]))
    with pytest.raises(world.WorldError, match="floating"):
        world.validate(_deck_world(pose=[0.21, 0, 0]))


@pytest.mark.parametrize("pkg,err", [
    ({"z": 0.5}, "floating"),                       # nothing of that height under it
    ({"z": 0.0}, "inside prop"),                    # floor level, but sunk into the deck
    ({"pose": [3, 0, 0]}, "floating"),              # off the deck
    ({"magnet_face": "front"}, "magnet_face"),
    ({"size": [0.2, 0, 0.1]}, "size"),
    ({"size": [0.2, 0.2]}, "size"),
    ({"id": "deck"}, "duplicate"),
    ({"id": "Bad"}, "id"),
    ({"colour": [1, 0, 0]}, "unknown key"),
    ({"z": -1}, "out of range"),
])
def test_package_validation_rejects(pkg, err):
    with pytest.raises(world.WorldError, match=err):
        world.validate(_deck_world(**pkg))


def test_too_many_packages():
    raw = {"name": "t", "packages": [{"id": f"p{i}", "pose": [i * 0.5, 0]}
                                     for i in range(world.MAX_PACKAGES + 1)]}
    with pytest.raises(world.WorldError, match="at most"):
        world.validate(raw)


@pytest.mark.parametrize("face", world.FACES)
def test_plate_sits_on_its_face_and_agrees_with_the_magnet_module(face):
    from simtest import magnet

    size = (0.24, 0.18, 0.12)
    psize, centre = world.plate_geometry(size, face)
    pose = {"x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0}
    face_centre, normal = magnet.magnet_face_pose(pose, size, face)
    # the plate's centre is half its thickness out from the face plane, along the normal
    off = [centre[i] - face_centre[i] for i in range(3)]
    assert off == pytest.approx([n * world.PLATE_THICKNESS_M / 2 for n in normal], abs=1e-12)
    # thin along the normal, 60% of the face across it
    thin = [abs(n) > 0.5 for n in normal]
    assert [s for s, t in zip(psize, thin) if t] == [world.PLATE_THICKNESS_M]
    across = [s / f for s, f, t in zip(psize, size, thin) if not t]
    assert across == pytest.approx([world.PLATE_FRACTION] * 2)


def test_top_z_of_each_prop_shape():
    w = world.validate({"name": "t", "props": [
        {"id": "b", "shape": "box", "size": [1, 1, 0.7], "pose": [0, 0]},
        {"id": "c", "shape": "cylinder", "radius": 0.3, "height": 0.9, "pose": [3, 0]},
        {"id": "u", "shape": "usd", "ref": "Isaac/Props/x.usd", "pose": [6, 0],
         "footprint": [1, 1], "height": 0.5}]})
    assert [world.top_z(p) for p in w["props"]] == [0.7, 0.9, 0.5]


# -- the committed pick worlds ---------------------------------------------------

PICK_WORLDS = ("pick_bench", "pick_orientations", "pick_heights", "pick_route")
# Worlds where Nav2 is not involved: the robot docks on its own from the spawn pose,
# one package after the other in file order, so every straight transit must be clear.
SELF_DOCK_WORLDS = ("pick_bench", "pick_orientations", "pick_heights")
STAGE_M = 0.35          # nodes/pick_planner.py backs out this far before a transit
CIRCLE_R = 0.26         # the chassis' swept radius, from its collision frame's corners


def _spawn():
    import yaml
    roster = yaml.safe_load((REPO_ROOT / "configs" / "robots.yaml").read_text())
    return tuple(next(r for r in roster["robots"] if r["id"] == "robot_0")["spawn"]["nvidia"][:2])


def _docks(w):
    from simtest import magnet
    out = []
    for pkg in w["packages"]:
        x, y, yaw = pkg["pose"]
        centre, normal = magnet.magnet_face_pose({"x": x, "y": y, "z": pkg["z"], "yaw": yaw},
                                                 pkg["size"], pkg["magnet_face"])
        out.append((pkg, centre, normal, magnet.dock_pose(centre, normal)))
    return out


@pytest.mark.parametrize("name", PICK_WORLDS)
def test_pick_world_plates_reachable_from_their_docks(name):
    from simtest import magnet
    for pkg, centre, normal, base in _docks(world.load(name)):
        px, py, pz = magnet.to_base(base, centre)
        assert px == pytest.approx(magnet.SHOULDER_XYZ_BASE[0] + magnet.REACH_NOM_M)
        assert py == pytest.approx(magnet.SHOULDER_XYZ_BASE[1])
        lo, hi = magnet.TCP_Z_RANGE_BASE
        assert lo < pz < hi, f"{name}/{pkg['id']}: plate at base z {pz:.2f} outside {lo}..{hi}"


@pytest.mark.parametrize("name", PICK_WORLDS)
def test_pick_world_docks_do_not_sit_in_props(name):
    from simtest import magnet
    w = world.load(name)
    hx, hy = magnet.CHASSIS_HALF_M
    for pkg, _, _, base in _docks(w):
        chassis = geometry.rect(base[0], base[1], base[2], 2 * hx, 2 * hy)
        for prop in w["props"]:
            gap = geometry.distance(chassis, world.prop_footprint(prop))
            assert gap > 0.01, f"{name}/{pkg['id']}: dock chassis touches {prop['id']}"


@pytest.mark.parametrize("name", SELF_DOCK_WORLDS)
def test_pick_world_transits_between_docks_are_clear(name):
    w = world.load(name)
    points = [_spawn()]
    for _, _, normal, base in _docks(w):
        points.append((base[0] + normal[0] * STAGE_M, base[1] + normal[1] * STAGE_M))
    for a, b in zip(points, points[1:]):
        n = max(2, int(math.dist(a, b) / 0.05))
        for k in range(n + 1):
            x, y = a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n
            for prop in w["props"]:
                assert geometry.distance(geometry.circle(x, y, CIRCLE_R),
                                         world.prop_footprint(prop)) > 0.02, \
                    f"{name}: transit {a}->{b} hits {prop['id']} near ({x:.2f}, {y:.2f})"


@pytest.mark.parametrize("name", ["pick_low", "pick_floor"])
def test_unreachable_worlds_really_are_below_the_envelope(name):
    from simtest import magnet
    for pkg, centre, _, base in _docks(world.load(name)):
        assert magnet.to_base(base, centre)[2] < magnet.TCP_Z_RANGE_BASE[0]
