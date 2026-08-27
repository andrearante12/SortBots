#!/usr/bin/env python3
"""Unit tests for scripts/maps_lib.py — the saved-map library core.

Pure python: no ROS, no Isaac, no GPU. Every test points SORTBOTS_MAPS_DIR at a
tmp_path, so nothing here can see or touch the repo's real maps/.

    /usr/bin/python3 -m pytest tests/maps_test.py

The SYSTEM python3 matters: conda's base env has no pytest, and it leads PATH
even in a "clean" shell — the same trap as ros2's python shebangs.
"""
from __future__ import annotations

import importlib
import json
import sqlite3
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "scripts"))


@pytest.fixture()
def maps(tmp_path, monkeypatch):
    """maps_lib with MAPS_DIR bound to tmp_path.

    MAPS_DIR is read at import time (like session.py's SESSIONS_DIR), so the env
    var has to be set BEFORE the reload — setting it afterwards would silently
    leave the module pointing at the repo's own maps/.
    """
    monkeypatch.setenv("SORTBOTS_MAPS_DIR", str(tmp_path))
    import maps_lib
    importlib.reload(maps_lib)
    assert maps_lib.MAPS_DIR == tmp_path
    yield maps_lib


# --------------------------------------------------------------------------
# Naming and path confinement
# --------------------------------------------------------------------------

@pytest.mark.parametrize("name", ["warehouse_full", "a", "map-2", "x9", "a" * 64])
def test_valid_names_resolve_inside_maps_dir(maps, name, tmp_path):
    assert maps.resolve_dir(name) == tmp_path / name


@pytest.mark.parametrize(
    "name",
    [
        "..",              # the classic
        "../etc",
        "a/b",             # a name is one path component, never a path
        "/etc",
        "",
        "Warehouse",       # uppercase: a name is also a directory name
        "-leading",        # must start alphanumeric
        ".hidden",
        "with space",
        "a" * 65,          # one over the cap
        "map.db",          # dots excluded so `.`/`..` can never be spelled
        None,
        7,
    ],
)
def test_bad_names_are_refused(maps, name):
    with pytest.raises(maps.MapError):
        maps.resolve_dir(name)


def test_symlink_escaping_maps_dir_is_refused(maps, tmp_path):
    """NAME_RX alone would pass this — the resolve()-and-confine check is why
    it fails. A symlink dropped inside maps/ escapes without the NAME looking
    suspicious at all."""
    outside = tmp_path.parent / "outside_the_library"
    outside.mkdir()
    (tmp_path / "escape").symlink_to(outside, target_is_directory=True)
    with pytest.raises(maps.MapError, match="outside"):
        maps.resolve_dir("escape")


def test_must_exist_reports_a_missing_map(maps):
    with pytest.raises(maps.MapError, match="no map named"):
        maps.resolve_dir("nope", must_exist=True)


# --------------------------------------------------------------------------
# Manifest roundtrip and validation
# --------------------------------------------------------------------------

def test_manifest_roundtrip(maps, tmp_path):
    m = maps.new_manifest("warehouse_full", title="Full warehouse", scene="nvidia")
    maps.write_manifest("warehouse_full", m)
    back = maps.read_manifest("warehouse_full")
    assert back["name"] == "warehouse_full"
    assert back["title"] == "Full warehouse"
    assert back["schema"] == maps.SCHEMA_VERSION
    assert back["db_state"] == "pending"
    assert back["created"].endswith("Z")


def test_write_is_atomic_and_leaves_no_tmp(maps, tmp_path):
    maps.write_manifest("m1", maps.new_manifest("m1"))
    leftovers = list((tmp_path / "m1").glob(".*tmp*"))
    assert leftovers == [], f"atomic write left {leftovers}"


def test_newer_schema_is_refused_rather_than_guessed(maps, tmp_path):
    m = maps.new_manifest("future")
    m["schema"] = maps.SCHEMA_VERSION + 1
    (tmp_path / "future").mkdir()
    (tmp_path / "future" / "map.json").write_text(json.dumps(m))
    with pytest.raises(maps.MapError, match="newer than this code understands"):
        maps.read_manifest("future")


def test_manifest_name_must_match_its_directory(maps, tmp_path):
    m = maps.new_manifest("claims_to_be_this")
    (tmp_path / "but_lives_here").mkdir()
    (tmp_path / "but_lives_here" / "map.json").write_text(json.dumps(m))
    with pytest.raises(maps.MapError, match="!= directory name"):
        maps.read_manifest("but_lives_here")


@pytest.mark.parametrize("key", ["schema", "name", "created", "dbs", "db_state"])
def test_missing_required_keys_are_named(maps, key):
    m = maps.new_manifest("m")
    del m[key]
    with pytest.raises(maps.MapError, match="missing required key"):
        maps.validate(m)


def test_db_entry_file_must_be_a_bare_filename(maps):
    m = maps.new_manifest("m")
    m["dbs"] = {"robot_0": {"file": "../../etc/passwd", "state": "complete"}}
    with pytest.raises(maps.MapError, match="bare filename"):
        maps.validate(m)


def test_bad_db_state_is_refused(maps):
    m = maps.new_manifest("m")
    m["dbs"] = {"robot_0": {"file": "map.db", "state": "probably_fine"}}
    with pytest.raises(maps.MapError, match="not in"):
        maps.validate(m)


# --------------------------------------------------------------------------
# Listing
# --------------------------------------------------------------------------

def test_list_maps_is_empty_without_a_directory(maps, tmp_path):
    (tmp_path).rmdir()
    assert maps.list_maps() == []


def test_list_maps_surfaces_a_broken_manifest_instead_of_dropping_it(maps, tmp_path):
    maps.write_manifest("good", maps.new_manifest("good"))
    (tmp_path / "broken").mkdir()
    (tmp_path / "broken" / "map.json").write_text("{not json")

    listed = {m["name"]: m for m in maps.list_maps()}
    assert set(listed) == {"good", "broken"}
    assert listed["good"]["status"] == "ready"
    assert listed["broken"]["status"] == "invalid"
    assert "not valid JSON" in listed["broken"]["error"]


def test_list_maps_ignores_directories_without_a_manifest(maps, tmp_path):
    (tmp_path / "just_a_dir").mkdir()
    assert maps.list_maps() == []


def test_list_maps_carries_an_absolute_db_path_for_the_picker(maps, tmp_path):
    maps.write_manifest("m", maps.new_manifest("m"))
    entry = maps.list_maps()[0]
    assert Path(entry["db_path"]).is_absolute()
    assert Path(entry["db_path"]).parent == tmp_path / "m"


# --------------------------------------------------------------------------
# Grid stats — pins map_coverage.classify_file's contract too
# --------------------------------------------------------------------------

def _write_grid(d: Path, pixels: bytes, w: int, h: int, res: float = 0.5) -> Path:
    d.mkdir(parents=True, exist_ok=True)
    (d / "grid.pgm").write_bytes(b"P5\n%d %d\n255\n" % (w, h) + pixels)
    yaml_path = d / "grid.yaml"
    yaml_path.write_text(
        f"image: grid.pgm\nmode: trinary\nresolution: {res}\n"
        "origin: [-1.0, -2.0, 0]\nnegate: 0\n"
        "occupied_thresh: 0.65\nfree_thresh: 0.25\n"
    )
    return yaml_path


def test_grid_stats_counts_trinary_sentinels_exactly(maps, tmp_path):
    # 4x4: 8 free (254), 4 unknown (205), 4 occupied (0). At 0.5 m cells each
    # cell is 0.25 m2, so free = 2.0 m2, occupied = 1.0 m2, known = 3.0 m2.
    pixels = bytes([254] * 8 + [205] * 4 + [0] * 4)
    yaml_path = _write_grid(tmp_path / "g", pixels, 4, 4)

    grid, coverage = maps.grid_stats(yaml_path)

    assert grid["width"] == 4 and grid["height"] == 4
    assert grid["resolution"] == 0.5
    assert grid["origin"] == [-1.0, -2.0, 0.0]
    assert grid["image"] == "grid.pgm", "image must stay relative for relocatability"
    assert coverage["free_m2"] == 2.0
    assert coverage["occupied_m2"] == 1.0
    assert coverage["known_m2"] == 3.0
    assert coverage["cells"] == {
        "free": 8, "occupied": 4, "unknown": 4, "known": 12, "total": 16,
    }


# --------------------------------------------------------------------------
# DB copying: the two-phase save, VACUUM, and the git-lfs pointer guard
# --------------------------------------------------------------------------

def _make_db(path: Path, rows: int = 2000) -> Path:
    conn = sqlite3.connect(path)
    conn.execute("create table ballast (id integer primary key, blob text)")
    conn.executemany(
        "insert into ballast (blob) values (?)", [("x" * 512,) for _ in range(rows)]
    )
    conn.commit()
    conn.close()
    return path


def test_copy_db_produces_a_readable_copy_with_a_stable_digest(maps, tmp_path):
    src = _make_db(tmp_path / "src.db")
    entry = maps.copy_db(src, tmp_path / "m" / "map.db", vacuum=False)

    assert entry["state"] == "complete"
    assert entry["file"] == "map.db"
    assert entry["bytes"] == (tmp_path / "m" / "map.db").stat().st_size
    assert entry["sha256"] == maps.sha256_file(tmp_path / "m" / "map.db")
    ok, detail = maps.integrity_ok(tmp_path / "m" / "map.db")
    assert ok, detail


def test_copy_db_vacuum_reclaims_space_from_a_dropped_table(maps, tmp_path):
    src = _make_db(tmp_path / "src.db")
    conn = sqlite3.connect(src)
    conn.execute("drop table ballast")
    conn.commit()
    conn.close()

    plain = maps.copy_db(src, tmp_path / "a" / "map.db", vacuum=False)
    vacuumed = maps.copy_db(src, tmp_path / "b" / "map.db", vacuum=True)

    assert vacuumed["vacuumed"] is True
    assert vacuumed["bytes"] < plain["bytes"]
    ok, detail = maps.integrity_ok(tmp_path / "b" / "map.db")
    assert ok, detail


def test_copy_db_leaves_no_temporaries_behind(maps, tmp_path):
    maps.copy_db(_make_db(tmp_path / "src.db"), tmp_path / "m" / "map.db")
    assert sorted(p.name for p in (tmp_path / "m").iterdir()) == ["map.db"]


def test_copy_db_refuses_a_missing_source(maps, tmp_path):
    with pytest.raises(maps.MapError, match="no database at"):
        maps.copy_db(tmp_path / "absent.db", tmp_path / "m" / "map.db")


def test_copy_db_refuses_an_unfetched_lfs_pointer(maps, tmp_path):
    pointer = tmp_path / "src.db"
    pointer.write_bytes(
        b"version https://git-lfs.github.com/spec/v1\noid sha256:deadbeef\nsize 12\n"
    )
    assert maps.is_lfs_pointer(pointer)
    with pytest.raises(maps.MapError, match="git lfs pull"):
        maps.copy_db(pointer, tmp_path / "m" / "map.db")


def test_two_phase_save_pending_then_complete(maps, tmp_path):
    """Grid captured mid-run, pose graph added after teardown."""
    m = maps.new_manifest("wh")
    maps.write_manifest("wh", m)
    assert maps.read_manifest("wh")["db_state"] == "pending"

    m["dbs"]["robot_0"] = maps.copy_db(
        _make_db(tmp_path / "src.db"), tmp_path / "wh" / "map.db"
    )
    m["db_state"] = maps.rollup_db_state(m["dbs"])
    maps.write_manifest("wh", m)

    assert maps.read_manifest("wh")["db_state"] == "complete"


def test_read_manifest_downgrades_a_deleted_db_to_missing(maps, tmp_path):
    m = maps.new_manifest("wh")
    m["dbs"]["robot_0"] = maps.copy_db(
        _make_db(tmp_path / "src.db"), tmp_path / "wh" / "map.db"
    )
    m["db_state"] = "complete"
    maps.write_manifest("wh", m)
    (tmp_path / "wh" / "map.db").unlink()

    assert maps.read_manifest("wh")["db_state"] == "missing"


def test_read_manifest_downgrades_an_lfs_pointer(maps, tmp_path):
    """A clone without git-lfs leaves a file that LOOKS present. The UI has to
    disable it rather than let RTAB-Map fail deep inside sqlite."""
    m = maps.new_manifest("wh")
    m["dbs"]["robot_0"] = maps.copy_db(
        _make_db(tmp_path / "src.db"), tmp_path / "wh" / "map.db"
    )
    m["db_state"] = "complete"
    maps.write_manifest("wh", m)
    (tmp_path / "wh" / "map.db").write_bytes(
        b"version https://git-lfs.github.com/spec/v1\noid sha256:beef\nsize 9\n"
    )

    assert maps.read_manifest("wh")["db_state"] == "pointer"


def test_rollup_reports_the_worst_state_across_a_fleet(maps):
    assert maps.rollup_db_state({}) == "pending"
    assert maps.rollup_db_state({"a": {"state": "complete"}}) == "complete"
    assert maps.rollup_db_state(
        {"a": {"state": "complete"}, "b": {"state": "pending"}}
    ) == "pending"
    assert maps.rollup_db_state(
        {"a": {"state": "pointer"}, "b": {"state": "missing"}}
    ) == "missing"


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-q"]))
