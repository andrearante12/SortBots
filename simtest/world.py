"""World layouts: what is in the warehouse besides the warehouse and the robots.

A world (configs/worlds/<name>.yaml) is the third layer of a test, under the
scenario (how the stack is launched) and the test case (what is asked of it
and how it is scored):

    world     props + scripted movers + graspable packages, placed in a named scene
    scenario  world + stack config (robots, map mode, explore, render knobs)
    test      scenario + goals + stop condition + metrics with pass criteria

It is read by THREE interpreters, which is why it is pure python + PyYAML:
  * scripts/spawn_warehouse.py (Isaac's venv) authors the prims and moves the
    actors every physics step, from pose_at() below;
  * webui/session.py (system python, no ROS) validates `world: NAME` in a
    scenario against this directory, so a scenario can only name a world that
    exists and parses;
  * the metrics (simtest/metrics) score against the same kinematics — e.g.
    map contamination needs the swept path of every actor, which is analytic
    here rather than reconstructed from a noisy message stream.

ACTOR TIME. Actors run on a WORLD CLOCK, not on raw sim time, so that a test
can put an actor's crossing and a robot's goal on the same t=0. Isaac starts
publishing long before the ROS stack is up — tens of sim-seconds that vary
with the real-time factor — so anchoring actors at sim t=0 would give a
different robot/actor phase every run. The world clock:
  * starts with the timeline when `autostart: true` (the live-demo default);
  * is reset/started by `/sim/world_cmd` (std_msgs/String, JSON
    {"cmd": "reset" | "start" | "pause"}), which the test probe sends at its
    t0 and the dashboard's actor controls send by hand.
`start_at_s` and every path timing below are world-clock seconds.

Fixed paths, deliberately: an actor never reacts to a robot. That keeps every
run of a test the same input, and it means a robot that yields correctly can
still be walked into — the clearance metric counts contact whoever moved.
Design paths so a robot that stops has somewhere to be.
"""
from __future__ import annotations

import math
import os
import re
from pathlib import Path

import yaml

from . import geometry

REPO_ROOT = Path(__file__).resolve().parent.parent
WORLDS_DIR = Path(os.environ.get("SORTBOTS_WORLDS_DIR") or REPO_ROOT / "configs" / "worlds")

# Same rule as maps_lib.NAME_RX: no `/` or `.`, so a name can never traverse
# out of WORLDS_DIR on its way to a path.
NAME_RX = re.compile(r"^[a-z0-9][a-z0-9_-]{0,63}$")
ID_RX = re.compile(r"^[a-z][a-z0-9_]{0,31}$")
# Asset-root-relative only (resolved by spawn_warehouse via get_assets_root_path,
# same as the warehouse). No absolute paths, URLs or `..`: a world file is
# reachable from the dashboard's launch form, so it must not be able to point
# Isaac at arbitrary files.
USD_REF_RX = re.compile(r"^Isaac/[A-Za-z0-9_][A-Za-z0-9_./-]*\.usdz?$")

SCENES = ("nvidia", "primitive")
SHAPES = ("box", "cylinder", "usd")
ACTOR_SHAPES = ("box", "cylinder")
MODES = ("once", "loop", "pingpong")
MAX_SPEED = 3.0          # m/s — a brisk human is ~1.4; anything faster is a typo
MAX_COORD = 200.0        # m — the NVIDIA warehouse is ~30 x 40
MAX_ITEMS = 64
MAX_PACKAGES = 16
# Which face of a package carries the magnet plate: the four sides in the
# package's own frame (+x is the way yaw 0 points), or the top.
FACES = ("+x", "-x", "+y", "-y", "top")
# A "very light amazon box", small enough that the SO-101's ~0.33 m horizontal
# reach (docs/manipulation.md) still has room for the box's own depth.
DEFAULT_PACKAGE_SIZE = (0.24, 0.18, 0.12)
DEFAULT_PACKAGE_COLOR = (0.76, 0.60, 0.42)   # cardboard
DEFAULT_MARK_COLOR = (0.16, 0.65, 0.35)
# The magnet plate. Both colours are authored by the sim and thresholded on by
# nodes/package_detector.py (configs/package_detector.yaml): change them together.
PLATE_COLOR = (0.05, 0.05, 0.06)
PLATE_THICKNESS_M = 0.004   # how far the plate stands proud of the box face
PLATE_FRACTION = 0.6        # the plate covers this fraction of the face's width and height
SUPPORT_EPS_M = 0.001    # a package's bottom must match its deck's top within this
SUPPORT_MARGIN_M = 0.03  # the package may overhang its deck by this much

DEFAULT_ACTOR_COLOR = (0.95, 0.45, 0.10)
DEFAULT_PROP_COLOR = (0.55, 0.42, 0.28)


class WorldError(ValueError):
    pass


# ---------------------------------------------------------------------------
# validation
# ---------------------------------------------------------------------------

def _num(v, what, lo=-math.inf, hi=math.inf, *, strict_lo=False) -> float:
    if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v):
        raise WorldError(f"{what}: expected a number, got {v!r}")
    v = float(v)
    if v < lo or v > hi or (strict_lo and v == lo):
        rng = f"{'(' if strict_lo else '['}{lo}, {hi}]"
        raise WorldError(f"{what}: {v} out of range {rng}")
    return v


def _xy(v, what) -> tuple[float, float]:
    if not isinstance(v, (list, tuple)) or len(v) != 2:
        raise WorldError(f"{what}: expected [x, y], got {v!r}")
    return (_num(v[0], f"{what}[0]", -MAX_COORD, MAX_COORD),
            _num(v[1], f"{what}[1]", -MAX_COORD, MAX_COORD))


def _pose(v, what) -> tuple[float, float, float]:
    if not isinstance(v, (list, tuple)) or len(v) not in (2, 3):
        raise WorldError(f"{what}: expected [x, y] or [x, y, yaw], got {v!r}")
    x, y = _xy(v[:2], what)
    yaw = _num(v[2], f"{what}[2]", -2 * math.pi, 2 * math.pi) if len(v) == 3 else 0.0
    return (x, y, yaw)


def _color(v, what, default) -> tuple[float, float, float]:
    if v is None:
        return default
    if not isinstance(v, (list, tuple)) or len(v) != 3:
        raise WorldError(f"{what}: expected [r, g, b] in 0..1")
    return tuple(_num(c, f"{what}", 0.0, 1.0) for c in v)


def _shape(raw: dict, what: str, allowed) -> dict:
    shape = raw.get("shape")
    if shape not in allowed:
        raise WorldError(f"{what}.shape: expected one of {list(allowed)}, got {shape!r}")
    out = {"shape": shape}
    if shape == "box":
        size = raw.get("size")
        if not isinstance(size, (list, tuple)) or len(size) != 3:
            raise WorldError(f"{what}.size: box needs [x, y, z] metres")
        out["size"] = tuple(_num(s, f"{what}.size", 0.0, 20.0, strict_lo=True) for s in size)
    elif shape == "cylinder":
        out["radius"] = _num(raw.get("radius"), f"{what}.radius", 0.0, 5.0, strict_lo=True)
        out["height"] = _num(raw.get("height"), f"{what}.height", 0.0, 10.0, strict_lo=True)
    else:  # usd
        ref = raw.get("ref")
        if not isinstance(ref, str) or not USD_REF_RX.match(ref) or ".." in ref:
            raise WorldError(f"{what}.ref: must be an asset-root path matching "
                             f"{USD_REF_RX.pattern} (no '..', no absolute paths)")
        out["ref"] = ref
        out["scale"] = _num(raw.get("scale", 1.0), f"{what}.scale", 0.0, 100.0, strict_lo=True)
        # The asset's own geometry is opaque to everything but Isaac, so the
        # metrics need to be told how much floor it covers.
        fp = raw.get("footprint")
        if not isinstance(fp, (list, tuple)) or len(fp) != 2:
            raise WorldError(f"{what}.footprint: usd props need [x, y] metres for the metrics")
        out["footprint"] = tuple(_num(s, f"{what}.footprint", 0.0, 20.0, strict_lo=True)
                                 for s in fp)
        out["height"] = _num(raw.get("height", 1.0), f"{what}.height", 0.0, 10.0,
                             strict_lo=True)
    return out


def _validate_path(raw, what) -> dict:
    if not isinstance(raw, dict):
        raise WorldError(f"{what}: expected a mapping")
    unknown = set(raw) - {"waypoints", "speed_mps", "mode", "start_at_s", "dwell_s"}
    if unknown:
        raise WorldError(f"{what}: unknown key(s) {sorted(unknown)}")
    wps = raw.get("waypoints")
    if not isinstance(wps, list) or len(wps) < 2:
        raise WorldError(f"{what}.waypoints: need at least two [x, y] points")
    pts = [_xy(p, f"{what}.waypoints[{i}]") for i, p in enumerate(wps)]
    for i in range(len(pts) - 1):
        if math.dist(pts[i], pts[i + 1]) < 1e-3:
            raise WorldError(f"{what}.waypoints[{i + 1}]: repeats the previous point "
                             "(use dwell_s to stand still)")
    mode = raw.get("mode", "pingpong")
    if mode not in MODES:
        raise WorldError(f"{what}.mode: expected one of {list(MODES)}, got {mode!r}")
    if mode == "loop" and math.dist(pts[0], pts[-1]) < 1e-3:
        raise WorldError(f"{what}.waypoints: a loop closes itself — don't repeat the first point")
    return {
        "waypoints": tuple(pts),
        "speed_mps": _num(raw.get("speed_mps", 1.0), f"{what}.speed_mps", 0.0, MAX_SPEED,
                          strict_lo=True),
        "mode": mode,
        "start_at_s": _num(raw.get("start_at_s", 0.0), f"{what}.start_at_s", 0.0, 3600.0),
        "dwell_s": _num(raw.get("dwell_s", 0.0), f"{what}.dwell_s", 0.0, 600.0),
    }


def _validate_item(raw, what, *, actor: bool) -> dict:
    if not isinstance(raw, dict):
        raise WorldError(f"{what}: expected a mapping")
    allowed = {"id", "shape", "size", "radius", "height", "color", "label"}
    allowed |= {"path"} if actor else {"pose", "ref", "scale", "footprint", "collider"}
    unknown = set(raw) - allowed
    if unknown:
        raise WorldError(f"{what}: unknown key(s) {sorted(unknown)}")
    iid = raw.get("id")
    if not isinstance(iid, str) or not ID_RX.match(iid):
        raise WorldError(f"{what}.id: must match {ID_RX.pattern}")
    out = {"id": iid, **_shape(raw, f"{what}", ACTOR_SHAPES if actor else SHAPES)}
    out["label"] = str(raw.get("label") or iid)[:40]
    out["color"] = _color(raw.get("color"), f"{what}.color",
                          DEFAULT_ACTOR_COLOR if actor else DEFAULT_PROP_COLOR)
    if actor:
        out["path"] = _validate_path(raw.get("path"), f"{what}.path")
    else:
        out["pose"] = _pose(raw.get("pose"), f"{what}.pose")
        collider = raw.get("collider", True)
        if not isinstance(collider, bool):
            raise WorldError(f"{what}.collider: expected a boolean")
        out["collider"] = collider
    return out


def _validate_package(raw, what) -> dict:
    if not isinstance(raw, dict):
        raise WorldError(f"{what}: expected a mapping")
    unknown = set(raw) - {"id", "size", "pose", "z", "magnet_face", "color", "label", "marker"}
    if unknown:
        raise WorldError(f"{what}: unknown key(s) {sorted(unknown)}")
    pid = raw.get("id")
    if not isinstance(pid, str) or not ID_RX.match(pid):
        raise WorldError(f"{what}.id: must match {ID_RX.pattern}")
    size = raw.get("size", DEFAULT_PACKAGE_SIZE)
    if not isinstance(size, (list, tuple)) or len(size) != 3:
        raise WorldError(f"{what}.size: expected [x, y, z] metres")
    face = raw.get("magnet_face", "+x")
    if face not in FACES:
        raise WorldError(f"{what}.magnet_face: expected one of {list(FACES)}, got {face!r}")
    size = tuple(_num(v, f"{what}.size", 0.0, 1.0, strict_lo=True) for v in size)
    marker = raw.get("marker")
    if marker is not None:
        from . import aruco

        if marker not in aruco.DATA:
            raise WorldError(f"{what}.marker: expected one of {sorted(aruco.DATA)}, got {marker!r}")
        if min(s for s in plate_geometry(size, face)[0] if s > PLATE_THICKNESS_M) < aruco.BACKING_M - 1e-9:
            raise WorldError(f"{what}.marker: the plate on face {face} is too small for a "
                             f"{aruco.BACKING_M * 100:g} cm marker")
    return {
        "id": pid,
        "size": size,
        "pose": _pose(raw.get("pose"), f"{what}.pose"),
        "z": _num(raw.get("z", 0.0), f"{what}.z", 0.0, 3.0),   # BOTTOM height
        "magnet_face": face,
        "marker": marker,       # the ArUco id printed on the plate, or None (simtest/aruco.py)
        "color": _color(raw.get("color"), f"{what}.color", DEFAULT_PACKAGE_COLOR),
        "label": str(raw.get("label") or pid)[:40],
    }


def top_z(item: dict) -> float:
    """Height of a prop's top surface above the floor (props stand on it)."""
    if item["shape"] == "box":
        return item["size"][2]
    return item["height"]


def package_footprint(pkg: dict) -> tuple:
    x, y, yaw = pkg["pose"]
    return geometry.rect(x, y, yaw, pkg["size"][0], pkg["size"][1])


def plate_geometry(size, face: str) -> tuple[tuple, tuple]:
    """The magnet plate as a box in the package's own frame: (size, centre), centre
    relative to the package's (x, y, bottom z). A thin slab on the named face, standing
    PLATE_THICKNESS_M proud of it. This is what scripts/spawn_warehouse.py authors and
    what nodes/package_vision.py looks for; its INNER face is the face plane that
    simtest.magnet.magnet_face_pose reports."""
    sx, sy, sz = size
    t, f = PLATE_THICKNESS_M, PLATE_FRACTION
    if face in ("+x", "-x"):
        return (t, f * sy, f * sz), (math.copysign(sx / 2 + t / 2, 1 if face == "+x" else -1),
                                     0.0, sz / 2)
    if face in ("+y", "-y"):
        return (f * sx, t, f * sz), (0.0, math.copysign(sy / 2 + t / 2, 1 if face == "+y" else -1),
                                     sz / 2)
    return (f * sx, f * sy, t), (0.0, 0.0, sz + t / 2)


def support_for(w: dict, pkg: dict) -> dict | None:
    """The prop a package stands on, or None when it stands on the floor.

    Raises WorldError for a layout that cannot exist: a package floating with
    nothing under it (or on a prop of a different height), or sunk into a prop.
    Packages never move by themselves, so this is checked once at load rather
    than left to the physics."""
    fp = package_footprint(pkg)
    x, y, _ = pkg["pose"]
    if pkg["z"] <= SUPPORT_EPS_M:
        for prop in w["props"]:
            if geometry.distance(fp, prop_footprint(prop)) <= 0.0:
                raise WorldError(f"package {pkg['id']!r}: sits on the floor inside prop "
                                 f"{prop['id']!r} — give it z: {top_z(prop):g} to put it on top")
        return None
    for prop in w["props"]:
        pf = prop_footprint(prop)
        if abs(top_z(prop) - pkg["z"]) > SUPPORT_EPS_M:
            continue
        if geometry.contains(pf, (x, y)) and all(
                geometry.contains(pf, c, SUPPORT_MARGIN_M) for c in fp[1]):
            return prop
    raise WorldError(f"package {pkg['id']!r}: floating at z={pkg['z']:g} — no prop of that "
                     f"height under it (props' tops: "
                     f"{sorted({round(top_z(p), 3) for p in w['props']})})")


def _validate_head(raw) -> dict | None:
    """Where the head starts, radians. Tilt > 0 looks down. Per world, because a tilted
    camera sees less of what is ahead: exploration and Nav2 keep theirs level."""
    if raw is None:
        return None
    if not isinstance(raw, dict) or set(raw) - {"pan", "tilt"}:
        raise WorldError("head: expected {pan, tilt} in radians")
    return {"pan": _num(raw.get("pan", 0.0), "head.pan", -1.57, 1.57),
            "tilt": _num(raw.get("tilt", 0.0), "head.tilt", -0.76, 1.45)}


def _validate_view(raw) -> dict | None:
    """Where the Sim camera starts in this world, instead of configs/sim_view.yaml's."""
    if raw is None:
        return None
    if not isinstance(raw, dict) or set(raw) != {"eye", "target"}:
        raise WorldError("view: expected {eye: [x, y, z], target: [x, y, z]}")
    out = {}
    for key in ("eye", "target"):
        v = raw[key]
        if not isinstance(v, (list, tuple)) or len(v) != 3:
            raise WorldError(f"view.{key}: expected [x, y, z]")
        out[key] = [_num(c, f"view.{key}", -200, 200) for c in v]
    return out


def _validate_mark(raw, what) -> dict:
    """A painted rectangle on a surface: where a place test wants a package set down.
    Nothing to collide with and nothing a package rests on: it only shows the spot."""
    if not isinstance(raw, dict):
        raise WorldError(f"{what}: expected a mapping")
    unknown = set(raw) - {"id", "size", "pose", "z", "color", "label"}
    if unknown:
        raise WorldError(f"{what}: unknown key(s) {sorted(unknown)}")
    mid = raw.get("id")
    if not isinstance(mid, str) or not ID_RX.match(mid):
        raise WorldError(f"{what}.id: must match {ID_RX.pattern}")
    size = raw.get("size")
    if not isinstance(size, (list, tuple)) or len(size) != 2:
        raise WorldError(f"{what}.size: expected [x, y] metres")
    return {
        "id": mid,
        "size": tuple(_num(v, f"{what}.size", 0.0, 5.0, strict_lo=True) for v in size),
        "pose": _pose(raw.get("pose"), f"{what}.pose"),
        "z": _num(raw.get("z", 0.0), f"{what}.z", 0.0, 3.0),     # the surface it is painted on
        "color": _color(raw.get("color"), f"{what}.color", DEFAULT_MARK_COLOR),
        "label": str(raw.get("label") or mid)[:40],
    }


def validate(raw, *, stem: str | None = None) -> dict:
    """A world mapping -> a normalized, fully-defaulted dict. Raises WorldError."""
    if not isinstance(raw, dict):
        raise WorldError("world file must contain a YAML mapping")
    unknown = set(raw) - {"name", "title", "description", "scenes", "autostart",
                          "props", "actors", "packages", "marks", "head", "view"}
    if unknown:
        raise WorldError(f"unknown key(s) {sorted(unknown)}")
    name = raw.get("name")
    if not isinstance(name, str) or not NAME_RX.match(name):
        raise WorldError(f"name: must match {NAME_RX.pattern}")
    if stem is not None and name != stem:
        raise WorldError(f"name: {name!r} must equal the filename stem {stem!r}")
    scenes = raw.get("scenes") or ["nvidia"]
    if not isinstance(scenes, list) or not scenes or set(scenes) - set(SCENES):
        raise WorldError(f"scenes: expected a non-empty subset of {list(SCENES)}")
    autostart = raw.get("autostart", True)
    if not isinstance(autostart, bool):
        raise WorldError("autostart: expected a boolean")

    items = {}
    out = {"name": name, "title": str(raw.get("title") or name),
           "description": str(raw.get("description") or "").strip(),
           "scenes": list(scenes), "autostart": autostart, "props": [], "actors": [],
           "packages": [], "marks": [], "head": _validate_head(raw.get("head")),
           "view": _validate_view(raw.get("view"))}
    for key, actor in (("props", False), ("actors", True)):
        lst = raw.get(key) or []
        if not isinstance(lst, list) or len(lst) > MAX_ITEMS:
            raise WorldError(f"{key}: expected a list of at most {MAX_ITEMS}")
        for i, item in enumerate(lst):
            v = _validate_item(item, f"{key}[{i}]", actor=actor)
            if v["id"] in items:
                raise WorldError(f"{key}[{i}].id: duplicate id {v['id']!r}")
            items[v["id"]] = v
            out[key].append(v)
    pk = raw.get("packages") or []
    if not isinstance(pk, list) or len(pk) > MAX_PACKAGES:
        raise WorldError(f"packages: expected a list of at most {MAX_PACKAGES}")
    for i, item in enumerate(pk):
        v = _validate_package(item, f"packages[{i}]")
        if v["id"] in items:
            raise WorldError(f"packages[{i}].id: duplicate id {v['id']!r}")
        items[v["id"]] = v
        out["packages"].append(v)
    for v in out["packages"]:
        support_for(out, v)     # raises on a floating or buried package
    mk = raw.get("marks") or []
    if not isinstance(mk, list) or len(mk) > MAX_PACKAGES:
        raise WorldError(f"marks: expected a list of at most {MAX_PACKAGES}")
    for i, item in enumerate(mk):
        v = _validate_mark(item, f"marks[{i}]")
        if v["id"] in items:
            raise WorldError(f"marks[{i}].id: duplicate id {v['id']!r}")
        items[v["id"]] = v
        out["marks"].append(v)
        # paint goes on something: the same check a package gets, as a flat package
        support_for(out, {**v, "size": (*v["size"], 0.01)})
    return out


def path_for(name: str) -> Path:
    if not isinstance(name, str) or not NAME_RX.match(name):
        raise WorldError(f"world name {name!r} must match {NAME_RX.pattern}")
    return WORLDS_DIR / f"{name}.yaml"


def load(name: str) -> dict:
    p = path_for(name)
    try:
        raw = yaml.safe_load(p.read_text())
    except FileNotFoundError:
        raise WorldError(f"no world named {name!r} (looked for {p})") from None
    except yaml.YAMLError as e:
        raise WorldError(f"{p.name}: {e}") from None
    return validate(raw, stem=p.stem)


def list_worlds() -> list[dict]:
    """Every configs/worlds/*.yaml, with invalid ones kept and explained —
    the same rule session.load_scenarios follows."""
    out = []
    for p in sorted(WORLDS_DIR.glob("*.yaml")):
        try:
            w = load(p.stem)
            out.append({"name": w["name"], "title": w["title"], "description": w["description"],
                        "scenes": w["scenes"], "props": len(w["props"]),
                        "actors": len(w["actors"]), "packages": len(w["packages"]),
                        "status": "ready"})
        except WorldError as e:
            out.append({"name": p.stem, "title": p.stem, "description": "", "scenes": [],
                        "props": 0, "actors": 0, "packages": 0, "status": "invalid",
                        "error": str(e)})
    return out


# ---------------------------------------------------------------------------
# actor kinematics
# ---------------------------------------------------------------------------

def _legs(path: dict) -> list[tuple[tuple, tuple]]:
    """The (start, end) move segments of one cycle."""
    pts = list(path["waypoints"])
    if path["mode"] == "loop":
        pts = pts + [pts[0]]
    elif path["mode"] == "pingpong":
        pts = pts + pts[-2::-1]
    return list(zip(pts[:-1], pts[1:]))


def _schedule(path: dict) -> tuple[list[tuple], float]:
    """[(t_start, t_end, a, b), ...] for one cycle, dwell segments as a == b."""
    sched, t = [], 0.0
    for a, b in _legs(path):
        if path["dwell_s"] > 0:
            sched.append((t, t + path["dwell_s"], a, a))
            t += path["dwell_s"]
        dur = math.dist(a, b) / path["speed_mps"]
        sched.append((t, t + dur, a, b))
        t += dur
    return sched, t


def cycle_s(path: dict) -> float:
    return _schedule(path)[1]


def pose_at(path: dict, t: float) -> dict:
    """Actor pose at world-clock time t: {x, y, yaw, vx, vy, moving}.

    Before start_at_s the actor waits at its first waypoint facing the first
    leg; `once` holds the last waypoint forever after one pass.
    """
    sched, period = _schedule(path)
    first_a, first_b = _legs(path)[0]
    yaw0 = math.atan2(first_b[1] - first_a[1], first_b[0] - first_a[0])
    rel = t - path["start_at_s"]
    if rel < 0 or period <= 0:
        x, y = path["waypoints"][0]
        return {"x": x, "y": y, "yaw": yaw0, "vx": 0.0, "vy": 0.0, "moving": False}
    if path["mode"] == "once" and rel >= period:
        a, b = _legs(path)[-1]
        return {"x": b[0], "y": b[1], "yaw": math.atan2(b[1] - a[1], b[0] - a[0]),
                "vx": 0.0, "vy": 0.0, "moving": False}
    if path["mode"] != "once":
        rel = math.fmod(rel, period)
    yaw = yaw0
    for t0, t1, a, b in sched:
        if a != b:
            yaw = math.atan2(b[1] - a[1], b[0] - a[0])
        if rel <= t1 or (t0, t1) == sched[-1][:2]:
            if a == b:
                return {"x": a[0], "y": a[1], "yaw": yaw, "vx": 0.0, "vy": 0.0,
                        "moving": False}
            f = 0.0 if t1 <= t0 else min(1.0, max(0.0, (rel - t0) / (t1 - t0)))
            s = path["speed_mps"]
            return {"x": a[0] + f * (b[0] - a[0]), "y": a[1] + f * (b[1] - a[1]),
                    "yaw": yaw, "vx": s * math.cos(yaw), "vy": s * math.sin(yaw),
                    "moving": True}
    raise AssertionError("unreachable: schedule exhausted")


# ---------------------------------------------------------------------------
# footprints (what the metrics measure against)
# ---------------------------------------------------------------------------

def footprint(item: dict, x: float, y: float, yaw: float) -> tuple:
    """The floor-plane shape of a prop or actor at a pose (simtest.geometry)."""
    if item["shape"] == "cylinder":
        return geometry.circle(x, y, item["radius"])
    if item["shape"] == "box":
        return geometry.rect(x, y, yaw, item["size"][0], item["size"][1])
    return geometry.rect(x, y, yaw, item["footprint"][0], item["footprint"][1])


def prop_footprint(prop: dict) -> tuple:
    x, y, yaw = prop["pose"]
    return footprint(prop, x, y, yaw)


def actor_footprint(actor: dict, t: float) -> tuple:
    p = pose_at(actor["path"], t)
    return footprint(actor, p["x"], p["y"], p["yaw"])


def half_extent(item: dict) -> float:
    """Radius of the smallest circle around the item's footprint."""
    if item["shape"] == "cylinder":
        return item["radius"]
    sx, sy = item["size"][:2] if item["shape"] == "box" else item["footprint"]
    return math.hypot(sx, sy) / 2.0


def swept_samples(actor: dict, step_m: float = 0.1) -> list[tuple[float, float]]:
    """Points along every leg the actor walks, `step_m` apart — the centreline
    of the floor it can ever occupy. Grow by half_extent() for the area."""
    out = []
    for a, b in _legs(actor["path"]):
        n = max(1, int(math.ceil(math.dist(a, b) / step_m)))
        out.extend((a[0] + (b[0] - a[0]) * i / n, a[1] + (b[1] - a[1]) * i / n)
                   for i in range(n + 1))
    return out
