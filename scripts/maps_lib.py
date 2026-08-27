#!/usr/bin/env python3
"""The saved-map library: manifest schema, listing, and artifact copying.

A *library map* is a named, committed snapshot of an explored environment,
living in `maps/<name>/`. It exists so a demo or test run can start from an
already-explored warehouse instead of re-exploring for twenty minutes:

    maps/<name>/
      map.json          # the manifest this module reads and writes
      grid.pgm          # occupancy grid, map_saver trinary output
      grid.yaml         # its metadata; `image:` stays RELATIVE (see below)
      map.db            # the primary robot's RTAB-Map pose graph (git-lfs)
      map_<rid>.db      # additional robots from a fleet save (git-lfs)

The GRID and the POSE GRAPH have different lifetimes, and that asymmetry is the
whole reason `db_state` exists. The grid can only be captured while the stack is
UP (it comes off the live fused /map topic); a plain copy of the sqlite pose
graph is only safe while the stack is DOWN, or via RTAB-Map's own backup service
which closes the file first. So `scripts/maps.sh save` is idempotent: run it
mid-run and you may get `db_state: "pending"` (grid in, pose graph not yet); run
it again after teardown and it promotes to `"complete"`.

Deliberately ROS-free — pure stdlib + PyYAML + numpy, same constraint as
webui/session.py, so it unit-tests offline (tests/maps_test.py) and imports
cleanly into serve.py's request path with no rclpy anywhere near it.

Grid metadata note: `grid.yaml`'s `image:` MUST stay relative.
scripts/map_coverage.py resolves a relative image against the yaml's own parent,
which is what keeps a `maps/<name>/` directory relocatable after a clone. An
absolute path baked in at save time would break on every other machine.
"""
from __future__ import annotations

import contextlib
import hashlib
import json
import os
import re
import sqlite3
import sys
from datetime import datetime, timezone
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent

# map_coverage.py is the single authority on turning a .yaml/.pgm pair into cell
# counts and areas — it defends against a subtlety this module must not
# re-derive (trinary sentinels vs. thresholds; see its classify_file docstring).
sys.path.insert(0, str(REPO_ROOT / "scripts"))
import map_coverage  # noqa: E402

# Overridable for the same reason session.py's SESSIONS_DIR is: tests point the
# whole chain (maps.sh, serve.py, session.py) at a temp dir instead of the
# repo's own maps/.
MAPS_DIR = Path(os.environ["SORTBOTS_MAPS_DIR"]) if os.environ.get("SORTBOTS_MAPS_DIR") \
    else REPO_ROOT / "maps"

# Lowercase-only, and no dots: a name becomes a directory name, an
# `<option value>` in the dashboard, and a shell word in `maps.sh save <name>`.
# Keeping it to this class means none of those three ever need quoting rules of
# their own, and it can't collide with `.` / `..`.
NAME_RX = re.compile(r"^[a-z0-9][a-z0-9_-]{0,63}$")

# Bumped only on a change that an older reader would MISREAD (not merely one it
# doesn't know about). read_manifest refuses anything higher rather than
# guessing, because the alternative is a demo silently loading the wrong DB.
SCHEMA_VERSION = 1

MANIFEST_NAME = "map.json"
GRID_STEM = "grid"

# git-lfs replaces a large file with a ~130-byte text pointer on a clone that
# doesn't have the extension installed. The file LOOKS present, so RTAB-Map
# opens it and fails deep inside sqlite. Catch it at the door instead.
LFS_POINTER_MAGIC = b"version https://git-lfs"

DB_STATES = ("pending", "complete", "pointer", "missing")


class MapError(ValueError):
    """Bad map name, bad manifest, or a refused library operation."""


def utcnow() -> str:
    """UTC ISO-8601 to the second, matching session.py's session-id timestamps."""
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


# --------------------------------------------------------------------------
# Naming and path confinement
# --------------------------------------------------------------------------

def resolve_dir(name: str, *, must_exist: bool = False) -> Path:
    """`maps/<name>/`, or raise. The only way to turn a request into a path.

    serve.py exposes `POST /api/map/save` on an interface that binds 0.0.0.0,
    so `name` is untrusted input. NAME_RX alone already excludes `/` and `..`,
    but the resolve()-and-confine check below is kept as well: NAME_RX could be
    loosened some day, and a symlink placed inside maps/ can escape MAPS_DIR
    without the name containing anything suspicious at all.
    """
    if not isinstance(name, str) or not NAME_RX.match(name):
        raise MapError(f"map name {name!r} does not match {NAME_RX.pattern}")
    root = MAPS_DIR.resolve()
    path = (root / name).resolve()
    if not _is_within(path, root):
        raise MapError(f"map name {name!r} resolves outside {root}")
    if must_exist and not (path / MANIFEST_NAME).is_file():
        raise MapError(f"no map named {name!r} in {root} (scripts/maps.sh list)")
    return path


def _is_within(path: Path, root: Path) -> bool:
    return path == root or root in path.parents


# --------------------------------------------------------------------------
# Manifest read / validate / write
# --------------------------------------------------------------------------

_REQUIRED = ("schema", "name", "created", "dbs", "db_state")


def validate(manifest: dict, *, name: str | None = None) -> dict:
    """Structural check. Returns the manifest so it can be used inline."""
    if not isinstance(manifest, dict):
        raise MapError(f"manifest must be a mapping, got {type(manifest).__name__}")

    missing = [k for k in _REQUIRED if k not in manifest]
    if missing:
        raise MapError(f"manifest missing required key(s) {missing}")

    schema = manifest["schema"]
    if not isinstance(schema, int) or schema < 1:
        raise MapError(f"schema must be a positive int, got {schema!r}")
    if schema > SCHEMA_VERSION:
        raise MapError(
            f"manifest schema {schema} is newer than this code understands "
            f"({SCHEMA_VERSION}) — update the repo rather than loading it blind"
        )

    if not NAME_RX.match(str(manifest["name"])):
        raise MapError(f"manifest name {manifest['name']!r} does not match {NAME_RX.pattern}")
    if name is not None and manifest["name"] != name:
        # Same rule as configs/scenarios/*.yaml: the name IS the address, so a
        # manifest disagreeing with its directory would make `maps.sh show X`
        # and `maps.sh rm X` operate on different things.
        raise MapError(f"manifest name {manifest['name']!r} != directory name {name!r}")

    if not isinstance(manifest["dbs"], dict):
        raise MapError("dbs must be a mapping of robot_id -> db entry")
    for rid, entry in manifest["dbs"].items():
        if not isinstance(entry, dict):
            raise MapError(f"dbs[{rid!r}] must be a mapping")
        for key in ("file", "state"):
            if key not in entry:
                raise MapError(f"dbs[{rid!r}] missing {key!r}")
        if entry["state"] not in DB_STATES:
            raise MapError(f"dbs[{rid!r}].state {entry['state']!r} not in {DB_STATES}")
        if "/" in str(entry["file"]) or str(entry["file"]).startswith("."):
            raise MapError(f"dbs[{rid!r}].file must be a bare filename, got {entry['file']!r}")

    if manifest["db_state"] not in DB_STATES:
        raise MapError(f"db_state {manifest['db_state']!r} not in {DB_STATES}")

    return manifest


def read_manifest(name: str, *, check_files: bool = True) -> dict:
    """Load, validate, and reconcile `maps/<name>/map.json` against the disk.

    `check_files` recomputes `db_state` from what is actually there, because the
    manifest records intent and the filesystem records fact, and they diverge in
    two ordinary ways: a clone without git-lfs leaves a pointer file, and
    someone can delete a .db by hand. Both must degrade to a disabled option in
    the UI rather than to a confusing RTAB-Map crash.
    """
    d = resolve_dir(name, must_exist=True)
    try:
        manifest = json.loads((d / MANIFEST_NAME).read_text())
    except json.JSONDecodeError as e:
        raise MapError(f"{d / MANIFEST_NAME}: not valid JSON ({e})") from e
    validate(manifest, name=name)
    if check_files:
        _reconcile(manifest, d)
    return manifest


def _reconcile(manifest: dict, d: Path) -> None:
    """Downgrade db entries whose file is missing or an unfetched LFS pointer."""
    for entry in manifest["dbs"].values():
        if entry["state"] != "complete":
            continue
        p = d / entry["file"]
        if not p.is_file():
            entry["state"] = "missing"
        elif is_lfs_pointer(p):
            entry["state"] = "pointer"
    manifest["db_state"] = rollup_db_state(manifest["dbs"])

    grid_yaml = d / f"{GRID_STEM}.yaml"
    manifest["has_grid"] = grid_yaml.is_file() and (d / f"{GRID_STEM}.pgm").is_file()


def rollup_db_state(dbs: dict) -> str:
    """One badge for the whole entry: the worst state any of its DBs is in.

    Ordered worst-first so a fleet map with one unfetched DB reads as "pointer"
    rather than as complete — the UI decides whether the entry is loadable from
    this single field.
    """
    if not dbs:
        return "pending"
    for state in ("missing", "pointer", "pending"):
        if any(e["state"] == state for e in dbs.values()):
            return state
    return "complete"


def write_manifest(name: str, manifest: dict) -> Path:
    """Validate then write atomically.

    Atomic because `maps.sh save` writes the manifest LAST, as the commit point
    of a multi-file operation: a reader must never see a manifest that claims a
    DB the copy hadn't finished writing. Same .tmp + os.replace pattern as
    SessionManager._persist().
    """
    d = resolve_dir(name)
    validate(manifest, name=name)
    d.mkdir(parents=True, exist_ok=True)
    target = d / MANIFEST_NAME
    tmp = d / f".{MANIFEST_NAME}.tmp"
    tmp.write_text(json.dumps(manifest, indent=2, ensure_ascii=False) + "\n")
    os.replace(tmp, target)
    return target


def new_manifest(name: str, **kw) -> dict:
    """A schema-correct empty entry: grid pending, no DBs yet."""
    manifest = {
        "schema": SCHEMA_VERSION,
        "name": name,
        "title": kw.get("title") or name,
        "description": kw.get("description") or "",
        "created": utcnow(),
        "updated": utcnow(),
        "scene": kw.get("scene", "nvidia"),
        "robot_ids": list(kw.get("robot_ids") or [kw.get("primary_robot_id", "robot_0")]),
        "primary_robot_id": kw.get("primary_robot_id", "robot_0"),
        "source": kw.get("source") or {},
        "grid": None,
        "coverage": None,
        "dbs": {},
        "db_state": "pending",
        "versions": versions(),
    }
    return manifest


# --------------------------------------------------------------------------
# Listing
# --------------------------------------------------------------------------

def list_maps() -> list[dict]:
    """Every entry in MAPS_DIR, newest first, invalid ones INCLUDED.

    A manifest that fails to load comes back as `status: "invalid"` with an
    `error` string rather than being dropped — same contract as
    session.py's load_scenarios(). Silently hiding a broken entry is how you
    get "my map disappeared" with nothing to go on; showing it with the reason
    attached is how you get it fixed.
    """
    root = MAPS_DIR
    if not root.is_dir():
        return []
    out: list[dict] = []
    for d in sorted(p for p in root.iterdir() if p.is_dir()):
        if not (d / MANIFEST_NAME).is_file():
            continue
        try:
            manifest = read_manifest(d.name)
            manifest["status"] = "ready"
        except MapError as e:
            manifest = {
                "schema": SCHEMA_VERSION,
                "name": d.name,
                "title": d.name,
                "status": "invalid",
                "error": str(e),
                "db_state": "missing",
                "dbs": {},
                "created": "",
            }
        manifest["db_path"] = str(db_path(d.name, manifest))
        out.append(manifest)
    out.sort(key=lambda m: m.get("created") or "", reverse=True)
    return out


def db_path(name: str, manifest: dict) -> Path:
    """Absolute path of the DB `--map` should be pointed at for this entry."""
    d = MAPS_DIR / name
    primary = manifest.get("primary_robot_id", "robot_0")
    entry = (manifest.get("dbs") or {}).get(primary)
    return d / (entry["file"] if entry else "map.db")


# --------------------------------------------------------------------------
# Artifacts: grid stats and DB copying
# --------------------------------------------------------------------------

def grid_stats(yaml_path: Path) -> tuple[dict, dict]:
    """(grid metadata, coverage) for a map_saver .yaml/.pgm pair.

    Coverage numbers come straight from map_coverage.classify_file — the areas
    are the defensible metric (nodes/explorer.py's own explored_pct is a
    convenience readout whose denominator grows as the map does).
    """
    yaml_path = Path(yaml_path)
    summary = map_coverage.classify_file(yaml_path)
    meta = yaml.safe_load(yaml_path.read_text())
    grid = {
        "image": str(meta["image"]),
        "yaml": yaml_path.name,
        "resolution": float(meta["resolution"]),
        "width": summary["width"],
        "height": summary["height"],
        "origin": [float(v) for v in meta.get("origin", [0.0, 0.0, 0.0])],
        "occupied_thresh": float(meta.get("occupied_thresh", 0.65)),
        "free_thresh": float(meta.get("free_thresh", 0.25)),
        "mode": meta.get("mode", "trinary"),
        "negate": int(meta.get("negate", 0)),
    }
    coverage = {
        "free_m2": summary["area_m2"]["free"],
        "occupied_m2": summary["area_m2"]["occupied"],
        "known_m2": summary["area_m2"]["known"],
        "extent_m2": summary["area_m2"]["extent"],
        "cells": summary["cells"],
    }
    return grid, coverage


def is_lfs_pointer(path: Path) -> bool:
    """True for an unfetched git-lfs pointer standing in for the real file."""
    try:
        with open(path, "rb") as f:
            return f.read(len(LFS_POINTER_MAGIC)) == LFS_POINTER_MAGIC
    except OSError:
        return False


def sha256_file(path: Path, chunk: int = 1 << 20) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for block in iter(lambda: f.read(chunk), b""):
            h.update(block)
    return h.hexdigest()


def copy_db(src: Path, dst: Path, *, vacuum: bool = True) -> dict:
    """Consistent copy of an RTAB-Map sqlite pose graph. Returns a `dbs` entry.

    Done in Python, never by shelling out to sqlite3(1): the sqlite3 on PATH
    here is conda's and /usr/bin/sqlite3 does not exist, which is the same
    PATH-ordering trap that bites ros2's python shebangs.

    `Connection.backup()` takes sqlite's own consistent snapshot even if the
    source is being written — belt and braces behind maps.sh's pgrep guard.
    VACUUM INTO then rewrites it compactly; these DBs are 150-500 MB and every
    saved entry is a tracked git-lfs object, so the reclaim is worth the extra
    pass. --no-vacuum exists for when save time matters more than bytes.
    """
    src, dst = Path(src), Path(dst)
    if not src.is_file():
        raise MapError(f"no database at {src}")
    if is_lfs_pointer(src):
        raise MapError(f"{src} is an unfetched git-lfs pointer — run `git lfs pull`")
    dst.parent.mkdir(parents=True, exist_ok=True)

    staged = dst.with_suffix(dst.suffix + ".tmp")
    for leftover in (staged, staged.with_suffix(".tmp.vac")):
        leftover.unlink(missing_ok=True)
    try:
        # closing(), not a bare `with`: sqlite3's context manager commits the
        # transaction but does NOT close the connection, and os.replace on a
        # file this process still holds open is asking for trouble on the
        # copies that follow.
        with contextlib.closing(sqlite3.connect(f"file:{src}?mode=ro", uri=True)) as source, \
                contextlib.closing(sqlite3.connect(staged)) as target:
            source.backup(target)

        if vacuum:
            vac = staged.with_suffix(".tmp.vac")
            vac.unlink(missing_ok=True)
            with contextlib.closing(sqlite3.connect(staged)) as conn:
                # VACUUM INTO writes a fresh, defragmented database rather than
                # rewriting in place, so a failure here leaves `staged` intact.
                conn.execute("VACUUM INTO ?", (str(vac),))
            os.replace(vac, staged)

        os.replace(staged, dst)
    finally:
        staged.unlink(missing_ok=True)
        staged.with_suffix(".tmp.vac").unlink(missing_ok=True)

    return {
        "file": dst.name,
        "bytes": dst.stat().st_size,
        "sha256": sha256_file(dst),
        "vacuumed": bool(vacuum),
        "state": "complete",
    }


def integrity_ok(path: Path) -> tuple[bool, str]:
    """sqlite `pragma integrity_check` — catches a torn or truncated copy."""
    if is_lfs_pointer(path):
        return False, "unfetched git-lfs pointer (run `git lfs pull`)"
    try:
        with contextlib.closing(sqlite3.connect(f"file:{path}?mode=ro", uri=True)) as conn:
            result = conn.execute("pragma integrity_check").fetchone()[0]
    except sqlite3.Error as e:
        return False, f"sqlite error: {e}"
    return result == "ok", result


def load_or_init(name: str, **kw) -> dict:
    """Existing manifest if there is one, otherwise a fresh empty entry.

    This is what makes `maps.sh save` idempotent: the mid-run call creates the
    entry with the grid, the post-teardown call finds it and adds the pose
    graph, and either one alone is a valid (if incomplete) library entry.
    """
    try:
        return read_manifest(name)
    except MapError as e:
        if "no map named" not in str(e):
            raise
        return new_manifest(name, **kw)


# --------------------------------------------------------------------------
# CLI — the manifest half of scripts/maps.sh
# --------------------------------------------------------------------------
#
# maps.sh owns everything that needs ROS (map_saver_cli, the rtabmap/backup
# service); this owns everything that needs the schema. Split that way so the
# schema stays testable without a running graph.

def _cmd_list(args) -> int:
    entries = list_maps()
    if args.json:
        print(json.dumps(entries, indent=2))
        return 0
    if not entries:
        print(f"no saved maps in {MAPS_DIR}  (scripts/maps.sh save NAME)")
        return 0
    for m in entries:
        if m.get("status") == "invalid":
            print(f"! {m['name']:<24} invalid  {m['error']}")
            continue
        cov = m.get("coverage") or {}
        free = f"{cov['free_m2']:.0f} m2" if cov.get("free_m2") is not None else "no grid"
        print(
            f"{_MARK.get(m['db_state'], ' ')} {m['name']:<24} {m['db_state']:<9} "
            f"{free:>9}  {m.get('created', '')[:10]}  {m.get('title', '')}"
        )
    return 0


# Mirrors session.py --list's marker column: a glyph so a scan of the list
# shows loadability without reading the state word.
_MARK = {"complete": "•", "pending": "·", "pointer": "!", "missing": "!"}


def _cmd_show(args) -> int:
    manifest = read_manifest(args.name)
    if args.json:
        print(json.dumps(manifest, indent=2))
        return 0
    print(f"{manifest['name']}  —  {manifest.get('title', '')}")
    if manifest.get("description"):
        print(f"  {manifest['description'].strip()}")
    print(f"  created    {manifest.get('created')}   updated {manifest.get('updated')}")
    print(f"  scene      {manifest.get('scene')}   robots {manifest.get('robot_ids')}")
    grid, cov = manifest.get("grid"), manifest.get("coverage")
    if grid and cov:
        print(f"  grid       {grid['width']}x{grid['height']} @ {grid['resolution']:.3f} m")
        print(f"  coverage   {cov['free_m2']:.1f} m2 free, {cov['known_m2']:.1f} m2 known")
    else:
        print("  grid       (none saved)")
    for rid, e in manifest["dbs"].items():
        print(f"  db {rid:<10} {e['state']:<9} {e['bytes'] / 1e6:.0f} MB  {e['file']}")
    print(f"  db_state   {manifest['db_state']}")
    print(f"  load with  scripts/sim_ctl.sh start library_localize "
          f"map={db_path(args.name, manifest)}")
    return 0


def _cmd_init(args) -> int:
    manifest = load_or_init(
        args.name,
        title=args.title,
        description=args.description,
        scene=args.scene,
        primary_robot_id=args.robot_id,
        robot_ids=args.robot_ids.split(",") if args.robot_ids else None,
        source=json.loads(args.source) if args.source else None,
    )
    # An existing entry keeps its `created` but takes new metadata, so a
    # re-save can correct a title without orphaning the entry.
    for key, value in (("title", args.title), ("description", args.description),
                       ("scene", args.scene)):
        if value:
            manifest[key] = value
    manifest["updated"] = utcnow()
    write_manifest(args.name, manifest)
    print(f"[maps] {args.name}: manifest ready ({MAPS_DIR / args.name})")
    return 0


def _cmd_add_grid(args) -> int:
    manifest = load_or_init(args.name)
    manifest["grid"], manifest["coverage"] = grid_stats(Path(args.yaml))
    manifest["updated"] = utcnow()
    write_manifest(args.name, manifest)
    cov = manifest["coverage"]
    print(f"[maps] {args.name}: grid saved — {cov['free_m2']:.1f} m2 free, "
          f"{cov['known_m2']:.1f} m2 known")
    return 0


def _cmd_add_db(args) -> int:
    manifest = load_or_init(args.name)
    rid = args.robot_id
    primary = manifest.get("primary_robot_id", "robot_0")
    # The primary robot's DB is the one `--map` points at, so it gets the
    # unadorned name; others are suffixed. Only the primary is loadable today —
    # sortbots_bringup.launch.py hardwires every non-primary robot's
    # database_path to ~/.ros/sortbots_<rid>.db to work around a launch-config
    # leak, so a fleet entry is save-many / load-one.
    filename = "map.db" if rid == primary else f"map_{rid}.db"
    entry = copy_db(Path(args.src), MAPS_DIR / args.name / filename, vacuum=not args.no_vacuum)
    manifest["dbs"][rid] = entry
    if rid not in manifest["robot_ids"]:
        manifest["robot_ids"].append(rid)
    manifest["db_state"] = rollup_db_state(manifest["dbs"])
    manifest["updated"] = utcnow()
    write_manifest(args.name, manifest)
    print(f"[maps] {args.name}: {rid} pose graph saved — {entry['bytes'] / 1e6:.0f} MB "
          f"({filename})")
    return 0


def _cmd_verify(args) -> int:
    manifest = read_manifest(args.name)
    d = MAPS_DIR / args.name
    rc = 0
    if manifest.get("grid"):
        pgm = d / manifest["grid"]["image"]
        if pgm.is_file():
            print(f"  grid       ok ({pgm.name})")
        else:
            print(f"  grid       MISSING ({pgm})")
            rc = 1
    else:
        print("  grid       (none saved)")

    for rid, e in manifest["dbs"].items():
        p = d / e["file"]
        if not p.is_file():
            print(f"  db {rid:<10} MISSING ({p})")
            rc = 1
            continue
        if is_lfs_pointer(p):
            print(f"  db {rid:<10} UNFETCHED git-lfs pointer — run `git lfs pull`")
            rc = 1
            continue
        digest = sha256_file(p)
        if e.get("sha256") and digest != e["sha256"]:
            print(f"  db {rid:<10} SHA MISMATCH (manifest {e['sha256'][:12]}, "
                  f"disk {digest[:12]})")
            rc = 1
            continue
        ok, detail = integrity_ok(p)
        print(f"  db {rid:<10} {'ok' if ok else 'CORRUPT: ' + detail}")
        rc = rc or (0 if ok else 1)
    print(f"[maps] {args.name}: {'ok' if rc == 0 else 'PROBLEMS FOUND'}")
    return rc


def _cmd_rm(args) -> int:
    import shutil
    d = resolve_dir(args.name, must_exist=True)
    shutil.rmtree(d)
    print(f"[maps] removed {d}")
    return 0


def main(argv=None) -> int:
    import argparse
    p = argparse.ArgumentParser(
        prog="maps_lib.py",
        description="Saved-map library manifest operations (the ROS-free half of "
                    "scripts/maps.sh — use that instead for ordinary work).",
    )
    sub = p.add_subparsers(dest="cmd", required=True)

    q = sub.add_parser("list"); q.add_argument("--json", action="store_true")
    q.set_defaults(fn=_cmd_list)

    q = sub.add_parser("show"); q.add_argument("name"); q.add_argument("--json", action="store_true")
    q.set_defaults(fn=_cmd_show)

    q = sub.add_parser("init")
    q.add_argument("name")
    q.add_argument("--title"); q.add_argument("--description")
    q.add_argument("--scene", default="nvidia")
    q.add_argument("--robot-id", default="robot_0")
    q.add_argument("--robot-ids", default="")
    q.add_argument("--source", help="JSON provenance blob")
    q.set_defaults(fn=_cmd_init)

    q = sub.add_parser("add-grid"); q.add_argument("name"); q.add_argument("--yaml", required=True)
    q.set_defaults(fn=_cmd_add_grid)

    q = sub.add_parser("add-db")
    q.add_argument("name")
    q.add_argument("--src", required=True)
    q.add_argument("--robot-id", default="robot_0")
    q.add_argument("--no-vacuum", action="store_true")
    q.set_defaults(fn=_cmd_add_db)

    q = sub.add_parser("verify"); q.add_argument("name"); q.set_defaults(fn=_cmd_verify)
    q = sub.add_parser("rm"); q.add_argument("name"); q.set_defaults(fn=_cmd_rm)

    args = p.parse_args(argv)
    try:
        return args.fn(args)
    except MapError as e:
        print(f"[maps] {e}", file=sys.stderr)
        return 1


def versions() -> dict:
    """Cheap provenance only.

    Isaac's version is deliberately NOT collected: reading it means sourcing
    activate_isaac.sh, which hard-refuses a ROS-sourced shell — and maps.sh runs
    in exactly such a shell. A null is more honest than a subshell that can't run.
    """
    rtabmap = None
    try:
        for p in Path("/opt/ros/jazzy/include").glob("rtabmap-*"):
            rtabmap = p.name.split("-", 1)[1]
            break
    except OSError:
        pass
    return {
        "rtabmap": rtabmap,
        "ros_distro": os.environ.get("ROS_DISTRO", "jazzy"),
        "isaac": None,
    }


if __name__ == "__main__":
    sys.exit(main())
