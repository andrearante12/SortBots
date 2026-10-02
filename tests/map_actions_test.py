"""Offline tests for the dashboard's clear/load map actions (no ROS).

The ROS service calls are replaced by a recorder; what is worth pinning is the
containment and ordering: names/ids are validated before anything runs, the
library db is only ever COPIED (RTAB-Map opens its file read-write), and odometry
is reset before the database is swapped.
"""
from __future__ import annotations

import sqlite3
import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "webui"))
sys.path.insert(0, str(REPO / "scripts"))

import maps_lib  # noqa: E402
import session as sm  # noqa: E402


@pytest.fixture()
def lib(tmp_path, monkeypatch):
    monkeypatch.setattr(maps_lib, "MAPS_DIR", tmp_path / "maps")
    monkeypatch.setattr(sm.maps_lib, "MAPS_DIR", tmp_path / "maps")
    monkeypatch.setenv("HOME", str(tmp_path / "home"))
    monkeypatch.setattr(sm.Path, "home", classmethod(lambda cls: tmp_path / "home"))
    calls = []
    monkeypatch.setattr(sm, "_ros_service_call",
                        lambda svc, typ, req, **kw: calls.append((svc, typ, req)) or "response")
    return tmp_path, calls


def _make_entry(root, name="aisle", with_db=True):
    d = root / "maps" / name
    d.mkdir(parents=True)
    m = maps_lib.new_manifest(name)
    if with_db:
        src = root / "src.db"
        con = sqlite3.connect(src)
        con.execute("create table t(x)")
        con.commit()
        con.close()
        m["dbs"]["robot_0"] = maps_lib.copy_db(src, d / "map.db", vacuum=False)
        m["db_state"] = maps_lib.rollup_db_state(m["dbs"])
    maps_lib.write_manifest(name, m)
    return d / "map.db"


def test_clear_resets_graph_then_odometry(lib):
    _, calls = lib
    sm.clear_map_blocking("robot_0")
    assert [c[0] for c in calls] == ["/robot_0/rtabmap/reset",
                                     "/robot_0/rgbd_odometry/reset_odom"]


@pytest.mark.parametrize("rid", ["robot 0", "robot_0; rm -rf /", ""])
def test_bad_robot_id_rejected_before_any_call(lib, rid):
    _, calls = lib
    with pytest.raises(maps_lib.MapError):
        sm.clear_map_blocking(rid)
    assert calls == []


def test_load_copies_never_points_at_library_file(lib):
    root, calls = lib
    src = _make_entry(root)
    before = src.read_bytes()
    out = sm.load_map_blocking("aisle", "robot_0")
    dst = Path(out["db"])
    assert dst != src and dst.exists() and dst.parent == root / "home" / ".ros"
    assert src.read_bytes() == before
    # odometry reset strictly before the db swap, and the swap targets the copy
    assert [c[0] for c in calls] == ["/robot_0/rgbd_odometry/reset_odom",
                                     "/robot_0/rtabmap/load_database"]
    assert str(dst) in calls[1][2] and "clear: false" in calls[1][2]


@pytest.mark.parametrize("name", ["../etc", "Bad Name", "a;b", ""])
def test_load_rejects_bad_names(lib, name):
    _, calls = lib
    with pytest.raises(maps_lib.MapError):
        sm.load_map_blocking(name)
    assert calls == []


def test_load_grid_only_entry_refused(lib):
    root, calls = lib
    _make_entry(root, with_db=False)
    with pytest.raises(maps_lib.MapError):
        sm.load_map_blocking("aisle")
    assert calls == []


def test_load_lfs_pointer_refused(lib):
    root, calls = lib
    src = _make_entry(root)
    src.write_bytes(maps_lib.LFS_POINTER_MAGIC + b"\noid sha256:0\nsize 1\n")
    with pytest.raises(maps_lib.MapError):
        sm.load_map_blocking("aisle")
    assert calls == []
