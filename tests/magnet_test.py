"""simtest.magnet: the capture rule, the snap, base placement. Offline, pure python.

    /usr/bin/python3 -m pytest tests/magnet_test.py
"""
from __future__ import annotations

import math
import random
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT))

from simtest import magnet, world  # noqa: E402

SIZE = (0.24, 0.18, 0.12)
FACES = ("+x", "-x", "+y", "-y", "top")


def pose(x=1.0, y=2.0, z=0.8, yaw=0.0):
    return {"x": x, "y": y, "z": z, "yaw": yaw}


def test_tcp_offset_matches_the_cad_parameters():
    """The TCP constant is the printed effector's 47.1 mm; recompute it from
    hardware/magnet_effector/params.py (pure dataclasses, no cadquery)."""
    sys.path.insert(0, str(REPO_ROOT / "hardware" / "magnet_effector"))
    from params import DEFAULT

    assert magnet.TCP_OFFSET_M[1] == pytest.approx(-DEFAULT.z_magnet_face / 1000.0, abs=5e-5)
    assert math.hypot(*magnet.APPROACH_LOCAL) == pytest.approx(1.0)


@pytest.mark.parametrize("yaw", [0.0, math.pi / 2, math.pi, -math.pi / 2, 0.7])
def test_face_pose_sides(yaw):
    p = pose(yaw=yaw)
    for face, (nx, ny) in {"+x": (1, 0), "-x": (-1, 0), "+y": (0, 1), "-y": (0, -1)}.items():
        centre, normal = magnet_face_pose_ok(p, face)
        # outward normal is the face direction rotated by yaw, horizontal
        c, s = math.cos(yaw), math.sin(yaw)
        assert normal == pytest.approx((c * nx - s * ny, s * nx + c * ny, 0.0), abs=1e-9)
        # the plate centre is half a box out from the centre, at mid height
        half = SIZE[0] / 2 if face in ("+x", "-x") else SIZE[1] / 2
        assert math.dist(centre[:2], (p["x"], p["y"])) == pytest.approx(half)
        assert centre[2] == pytest.approx(p["z"] + SIZE[2] / 2)


def magnet_face_pose_ok(p, face):
    return magnet.magnet_face_pose(p, SIZE, face)


def test_face_pose_top():
    centre, normal = magnet.magnet_face_pose(pose(yaw=0.4), SIZE, "top")
    assert normal == (0.0, 0.0, 1.0)
    assert centre == pytest.approx((1.0, 2.0, 0.8 + SIZE[2]))


def test_capture_accepts_and_rejects_at_the_tolerances():
    centre, normal = (0.0, 0.0, 0.8), (-1.0, 0.0, 0.0)   # plate faces -x
    along = (1.0, 0.0, 0.0)                              # so the magnet comes along +x

    def at(dx, deg=0.0):
        a = math.radians(deg)
        return magnet.capture((centre[0] + dx, 0.0, 0.8), (math.cos(a), math.sin(a), 0.0),
                              centre, normal)

    assert at(0.0)[0]
    assert at(magnet.CAPTURE_M - 0.001)[0]
    assert not at(magnet.CAPTURE_M + 0.001)[0]
    assert at(0.0, magnet.CAPTURE_DEG - 1)[0]
    assert not at(0.0, magnet.CAPTURE_DEG + 1)[0]
    # approaching the plate's back (pointing the wrong way) never captures
    ok, _, ang = magnet.capture(centre, (-1.0, 0.0, 0.0), centre, normal)
    assert not ok and ang == pytest.approx(180.0)
    assert along  # documented direction


@pytest.mark.parametrize("face", FACES)
def test_held_pose_round_trips_onto_the_tcp(face):
    rng = random.Random(1)
    for _ in range(25):
        az = rng.uniform(-math.pi, math.pi)
        el = 0.0 if face != "top" else -math.pi / 2
        if face != "top":
            el = rng.uniform(-0.3, 0.3)
        approach = (math.cos(el) * math.cos(az), math.cos(el) * math.sin(az), math.sin(el))
        tcp = (rng.uniform(-1, 1), rng.uniform(-1, 1), rng.uniform(0.6, 1.2))
        held = magnet.held_pose(tcp, approach, SIZE, face, 0.3)
        centre, normal = magnet.magnet_face_pose(held, SIZE, face)
        assert centre == pytest.approx(tcp, abs=1e-9)
        if face != "top":
            # normal is anti-parallel to the approach's horizontal part
            h = math.hypot(approach[0], approach[1])
            assert normal[:2] == pytest.approx((-approach[0] / h, -approach[1] / h), abs=1e-9)
        else:
            assert normal == (0.0, 0.0, 1.0)


def test_held_pose_keeps_yaw_when_pointing_straight_down():
    held = magnet.held_pose((0, 0, 1.0), (0.0, 0.0, -1.0), SIZE, "+x", 1.25)
    assert held["yaw"] == 1.25


@pytest.mark.parametrize("nx,ny", [(1, 0), (0, 1), (-1, 0), (0, -1), (0.6, 0.8), (-0.8, -0.6)])
def test_dock_pose_puts_the_plate_at_reach_on_the_radial_line(nx, ny):
    centre = (3.0, -1.5, 0.755)
    base = magnet.dock_pose(centre, (nx, ny, 0.0))
    # the base faces the plate head-on ...
    assert (math.cos(base[2]), math.sin(base[2])) == pytest.approx((-nx, -ny), abs=1e-9)
    # ... and the plate is REACH_NOM ahead of the shoulder, level with it laterally
    p = magnet.to_base(base, centre)
    assert p[0] == pytest.approx(magnet.SHOULDER_XYZ_BASE[0] + magnet.REACH_NOM_M)
    assert p[1] == pytest.approx(magnet.SHOULDER_XYZ_BASE[1])
    assert p[2] == pytest.approx(0.755 - magnet.BASE_Z_M)


def test_frame_round_trip():
    base = (2.0, -3.0, 0.9)
    p = (4.1, -2.2, 0.77)
    assert magnet.to_map(base, magnet.to_base(base, p)) == pytest.approx(p)
    v = (0.3, -0.5, 0.2)
    assert magnet.dir_to_map(base, magnet.dir_to_base(base, v)) == pytest.approx(v)


def _w():
    return world.validate({
        "name": "t",
        "props": [{"id": "deck", "shape": "box", "size": [0.5, 0.4, 0.8], "pose": [0, 0, 0]}],
    })


def test_support_top_z():
    w = _w()
    assert magnet.support_top_z(w, 0.0, 0.0) == pytest.approx(0.8)
    assert magnet.support_top_z(w, 3.0, 0.0) == 0.0
    assert magnet.support_top_z(None, 0.0, 0.0) == 0.0


def _sim():
    w = _w()
    pk = {"a": {"pose": pose(x=0.0, y=0.0, z=0.8), "size": SIZE, "face": "+x"},
          "b": {"pose": pose(x=0.0, y=1.5, z=0.0), "size": SIZE, "face": "+x"}}
    return magnet.MagnetSim(pk, lambda x, y: magnet.support_top_z(w, x, y)), pk


def _aim_at(sim, pid, standoff=0.0):
    centre, normal = magnet.magnet_face_pose(sim.pose(pid), SIZE, "+x")
    tcp = tuple(centre[i] + normal[i] * standoff for i in range(3))
    return tcp, (-normal[0], -normal[1], -normal[2])


def test_magnetsim_grabs_the_in_range_package_only_when_energized():
    sim, _ = _sim()
    tcp, ap = _aim_at(sim, "a", 0.01)
    d = sim.step(tcp, ap, False)
    assert d["held"] is None and d["nearest"]["id"] == "a"
    d = sim.step(tcp, ap, True)
    assert d["held"] == "a" and d["moved"] == ["a"]


def test_magnetsim_ignores_out_of_range_and_wrong_way():
    sim, _ = _sim()
    far, ap = _aim_at(sim, "a", 0.10)
    assert sim.step(far, ap, True)["held"] is None
    tcp, ap = _aim_at(sim, "a", 0.01)
    assert sim.step(tcp, (-ap[0], -ap[1], -ap[2]), True)["held"] is None


def test_magnetsim_carries_then_releases_onto_the_support():
    sim, _ = _sim()
    tcp, ap = _aim_at(sim, "a", 0.01)
    sim.step(tcp, ap, True)
    # carry it over open floor: it rides the TCP
    tcp2 = (2.0, 2.0, 1.0)
    d = sim.step(tcp2, ap, True)
    assert d["held"] == "a"
    centre, _ = magnet.magnet_face_pose(sim.pose("a"), SIZE, "+x")
    assert centre == pytest.approx(tcp2, abs=1e-9)
    # de-energize over the floor: it falls to z=0
    d = sim.step(tcp2, ap, False)
    assert d["held"] is None and d["moved"] == ["a"]
    assert sim.pose("a")["z"] == 0.0
    # and stays put afterwards
    assert sim.step(tcp2, ap, False)["moved"] == []


def test_magnetsim_release_over_the_deck_lands_on_top():
    sim, _ = _sim()
    sim.pk["b"]["pose"].update(x=0.0, y=0.0, z=1.0)      # pretend b is already held above the deck
    sim.held = "b"
    tcp, ap = _aim_at(sim, "b")
    sim.step(tcp, ap, False)
    assert sim.pose("b")["z"] == pytest.approx(0.8)
