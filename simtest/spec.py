"""Test cases: configs/tests/<name>.yaml — one reproducible, scored question.

    name: crossing_yield
    title: Walker crosses the aisle mouth — yield and arrive
    scenario: moving_obstacles          # configs/scenarios/ — how the stack runs
    overrides: {world: aisle_crossing}  # only keys that scenario lists under overrides:
    warmup_s: 5                         # sim-s between "stack ready" and t0
    goals:                              # Nav2 goals the probe sends, per robot in order
      - robot: robot_0
        pose: [-8.0, 3.0, -1.57]        # or station: shelf_a (configs/waypoints.yaml)
        at_s: 0                         # NO EARLIER than this (s after t0); default:
                                        # as soon as this robot's previous goal ends
        timeout_s: 120                  # cancelled + scored `timeout` after this
    tasks:                              # optional task_manager deliveries
      - {robot: robot_0, pickup: shelf_a, dropoff: zone_a, at_s: 10}
    stop:
      sim_s: 180                        # hard cap, sim seconds after t0
      until: [goals_done]               # goals_done | tasks_done | explorer_done
      settle_s: 3                       # keep scoring this long after `until` is met
    metrics:                            # simtest/metrics — name, params, pass criteria
      - metric: clearance
        pass: {contacts: {eq: 0}}
    repeats: 3
    pass_rate: 1.0                      # fraction of repeats (per variant) that must pass
    variants:                           # optional: the same test under other settings
      - {label: head_on, set: {overrides.world: aisle_head_on}}
    record: [dyn]                       # extra event streams to keep (simtest/events.py)

The three layers stay separate on purpose: a world is WHAT is in the
warehouse, a scenario is HOW the stack runs (and is what the dashboard
launches by hand for a live demo), a test is WHAT IS ASKED and HOW IT IS
SCORED. A test can only change what its scenario already lets the dashboard
change (`overrides:`), so nothing here widens what can reach a shell —
session.py's RUN_FLAGS allowlist stays the single gate.

t0 is when the probe restarts the world clock (simtest/world.py "ACTOR
TIME"); every `at_s`, `sim_s` and event time is measured from it.
"""
from __future__ import annotations

import copy
import math
import os
import re
import sys
from pathlib import Path

import yaml

from . import metrics as metrics_lib
from . import robots as robots_lib
from . import world as world_lib

REPO_ROOT = Path(__file__).resolve().parent.parent
TESTS_DIR = Path(os.environ.get("SORTBOTS_TESTS_DIR") or REPO_ROOT / "configs" / "tests")
RESULTS_DIR = Path(os.environ.get("SORTBOTS_TEST_RESULTS") or REPO_ROOT / "data" / "simtests")
WAYPOINTS = REPO_ROOT / "configs" / "waypoints.yaml"

NAME_RX = world_lib.NAME_RX
UNTIL = ("goals_done", "tasks_done", "explorer_done")
MAX_SIM_S = 3600.0
MAX_REPEATS = 50
MAX_VARIANTS = 20
TOP_KEYS = {"name", "title", "description", "scenario", "overrides", "warmup_s", "goals",
            "tasks", "stop", "metrics", "repeats", "pass_rate", "variants", "record"}
# What a variant may change. Not `scenario` or `metrics`: a variant is the
# same question asked under different conditions, and changing the scoring
# would make the variants' rows in one report incomparable.
VARIANT_KEYS_RX = re.compile(r"^(overrides\.[a-z_]+|stop\.(sim_s|settle_s)|warmup_s)$")


class SpecError(ValueError):
    pass


def _session():
    """webui/session.py — ROS-free by design, and the owner of the scenario
    allowlist. Imported lazily so simtest stays importable from Isaac's venv."""
    sys.path.insert(0, str(REPO_ROOT / "webui"))
    import session  # noqa: E402

    return session


def _stations() -> dict[str, dict]:
    with open(WAYPOINTS) as f:
        return (yaml.safe_load(f) or {}).get("stations") or {}


def _num(v, what, lo, hi) -> float:
    if isinstance(v, bool) or not isinstance(v, (int, float)) or not lo <= v <= hi:
        raise SpecError(f"{what}: expected a number in [{lo}, {hi}], got {v!r}")
    return float(v)


def _set_dotted(d: dict, dotted: str, value) -> None:
    keys = dotted.split(".")
    for k in keys[:-1]:
        d = d.setdefault(k, {})
    d[keys[-1]] = value


# ---------------------------------------------------------------------------
# validation
# ---------------------------------------------------------------------------

def _resolve_run(raw: dict) -> tuple[dict, dict]:
    """(scenario, run) after overrides, through session.py's own allowlist."""
    s = _session()
    try:
        scen = s.get_scenario(str(raw.get("scenario")))
    except s.ScenarioError as e:
        raise SpecError(f"scenario: {e}") from None
    if scen["status"] != "ready":
        raise SpecError(f"scenario {scen['name']!r} is {scen['status']!r}: "
                        f"{scen.get('error', 'not runnable')}")
    if scen["platform"] != "sim":
        raise SpecError(f"scenario {scen['name']!r} is platform {scen['platform']!r}; "
                        "tests run in the sim")
    overrides = raw.get("overrides") or {}
    if not isinstance(overrides, dict):
        raise SpecError("overrides: expected a mapping of scenario run keys")
    try:
        run = s.apply_overrides(scen, overrides)
    except s.ScenarioError as e:
        raise SpecError(f"overrides: {e}") from None
    return scen, run


def robots_for(run: dict) -> list[str]:
    if run.get("robot_ids"):
        return [r for r in str(run["robot_ids"]).split(",") if r]
    return robots_lib.roster()[: int(run.get("robots") or 1)]


def _goals(raw, robots: list[str], stations: dict) -> list[dict]:
    if raw is None:
        return []
    if not isinstance(raw, list):
        raise SpecError("goals: expected a list")
    out = []
    for i, g in enumerate(raw):
        w = f"goals[{i}]"
        if not isinstance(g, dict):
            raise SpecError(f"{w}: expected a mapping")
        unknown = set(g) - {"robot", "pose", "station", "at_s", "timeout_s"}
        if unknown:
            raise SpecError(f"{w}: unknown key(s) {sorted(unknown)}")
        if g.get("robot") not in robots:
            raise SpecError(f"{w}.robot: {g.get('robot')!r} is not in this run ({robots})")
        if ("pose" in g) == ("station" in g):
            raise SpecError(f"{w}: give exactly one of pose: [x, y, yaw] or station: NAME")
        if "station" in g:
            st = stations.get(g["station"])
            if st is None:
                raise SpecError(f"{w}.station: {g['station']!r} not in configs/waypoints.yaml")
            p = st["pose"]
            pose = [float(p["x"]), float(p["y"]), float(p.get("yaw", 0.0))]
        else:
            p = g["pose"]
            if not isinstance(p, list) or len(p) not in (2, 3):
                raise SpecError(f"{w}.pose: expected [x, y] or [x, y, yaw]")
            pose = [_num(p[0], f"{w}.pose[0]", -200, 200), _num(p[1], f"{w}.pose[1]", -200, 200),
                    _num(p[2], f"{w}.pose[2]", -2 * math.pi, 2 * math.pi) if len(p) == 3
                    else 0.0]
        out.append({"id": f"g{i}", "robot": g["robot"], "pose": pose,
                    "station": g.get("station"),
                    "at_s": _num(g.get("at_s", 0.0), f"{w}.at_s", 0, MAX_SIM_S),
                    "timeout_s": _num(g.get("timeout_s", 300.0), f"{w}.timeout_s",
                                      1, MAX_SIM_S)})
    return out


def _tasks(raw, robots: list[str], stations: dict) -> list[dict]:
    if raw is None:
        return []
    if not isinstance(raw, list):
        raise SpecError("tasks: expected a list")
    out = []
    for i, t in enumerate(raw):
        w = f"tasks[{i}]"
        if not isinstance(t, dict) or set(t) - {"robot", "pickup", "dropoff", "at_s"}:
            raise SpecError(f"{w}: expected {{robot, pickup, dropoff, at_s}}")
        if t.get("robot") not in robots:
            raise SpecError(f"{w}.robot: {t.get('robot')!r} is not in this run ({robots})")
        for key, kind in (("pickup", "pickup"), ("dropoff", "dropoff")):
            st = stations.get(t.get(key))
            if st is None:
                raise SpecError(f"{w}.{key}: {t.get(key)!r} not in configs/waypoints.yaml")
            if st.get("kind") != kind:
                raise SpecError(f"{w}.{key}: station {t[key]!r} is a {st.get('kind')!r}")
        out.append({"id": f"t{i}", "robot": t["robot"], "pickup": t["pickup"],
                    "dropoff": t["dropoff"],
                    "at_s": _num(t.get("at_s", 0.0), f"{w}.at_s", 0, MAX_SIM_S)})
    return out


def _stop(raw, goals, tasks, explore: bool) -> dict:
    if not isinstance(raw, dict) or set(raw) - {"sim_s", "until", "settle_s"}:
        raise SpecError("stop: expected {sim_s, until?, settle_s?}")
    until = raw.get("until") or []
    if not isinstance(until, list) or set(until) - set(UNTIL):
        raise SpecError(f"stop.until: expected a list from {list(UNTIL)}")
    if "goals_done" in until and not goals:
        raise SpecError("stop.until: goals_done needs at least one goal")
    if "tasks_done" in until and not tasks:
        raise SpecError("stop.until: tasks_done needs at least one task")
    if "explorer_done" in until and not explore:
        raise SpecError("stop.until: explorer_done needs a scenario that explores")
    return {"sim_s": _num(raw.get("sim_s"), "stop.sim_s", 1, MAX_SIM_S),
            "until": list(until),
            "settle_s": _num(raw.get("settle_s", 0.0), "stop.settle_s", 0, 600)}


def _resolve_one(raw: dict) -> dict:
    """Validate one variant (or the base) end to end, including metric context."""
    scen, run = _resolve_run(raw)
    robots = robots_for(run)
    stations = _stations()
    world = world_lib.load(run["world"]) if run.get("world") else None
    goals = _goals(raw.get("goals"), robots, stations)
    tasks = _tasks(raw.get("tasks"), robots, stations)
    stop = _stop(raw.get("stop"), goals, tasks, bool(run.get("explore")))
    try:
        entries = metrics_lib.validate_list(raw.get("metrics"))
    except metrics_lib.MetricError as e:
        raise SpecError(str(e)) from None
    resolved = {
        "scenario": scen["name"], "overrides": dict(raw.get("overrides") or {}),
        "run": run, "robots": robots, "world": world, "goals": goals, "tasks": tasks,
        "stop": stop, "metrics": entries,
        "warmup_s": _num(raw.get("warmup_s", 5.0), "warmup_s", 0, 600),
    }
    try:
        metrics_lib.MetricSet(entries, context(resolved))  # check_context() per metric
    except metrics_lib.MetricError as e:
        raise SpecError(str(e)) from None
    return resolved


def validate(raw, *, stem: str | None = None) -> dict:
    """A test-case mapping -> {name, title, ..., variants: [(label, resolved)]}."""
    if not isinstance(raw, dict):
        raise SpecError("test file must contain a YAML mapping")
    unknown = set(raw) - TOP_KEYS
    if unknown:
        raise SpecError(f"unknown key(s) {sorted(unknown)} (allowed: {sorted(TOP_KEYS)})")
    name = raw.get("name")
    if not isinstance(name, str) or not NAME_RX.match(name):
        raise SpecError(f"name: must match {NAME_RX.pattern}")
    if stem is not None and name != stem:
        raise SpecError(f"name: {name!r} must equal the filename stem {stem!r}")
    reps = raw.get("repeats", 1)
    if isinstance(reps, bool) or not isinstance(reps, int) or not 1 <= reps <= MAX_REPEATS:
        raise SpecError(f"repeats: integer 1..{MAX_REPEATS}")
    pass_rate = _num(raw.get("pass_rate", 1.0), "pass_rate", 0.0, 1.0)
    record = raw.get("record") or []
    from .events import STREAMS
    if not isinstance(record, list) or set(record) - set(STREAMS):
        raise SpecError(f"record: expected stream names from {sorted(STREAMS)}")

    named = raw.get("variants") or []
    if not isinstance(named, list) or len(named) > MAX_VARIANTS:
        raise SpecError(f"variants: a list of at most {MAX_VARIANTS} {{label, set}}")
    variants, labels = [], set()
    if not named:
        variants.append(("base", _resolve_one(raw)))
    for i, v in enumerate(named):
        w = f"variants[{i}]"
        if not isinstance(v, dict) or set(v) - {"label", "set"} or \
                not isinstance(v.get("label"), str) or not isinstance(v.get("set") or {}, dict):
            raise SpecError(f"{w}: expected {{label: str, set: {{dotted.key: value}}}}")
        if v["label"] in labels or not re.match(r"^[A-Za-z0-9_.=-]{1,40}$", v["label"]):
            raise SpecError(f"{w}.label: must be unique, 1-40 of [A-Za-z0-9_.=-]")
        labels.add(v["label"])
        body = copy.deepcopy({k: raw[k] for k in raw if k != "variants"})
        for key, value in (v.get("set") or {}).items():
            if not VARIANT_KEYS_RX.match(key):
                raise SpecError(f"{w}.set.{key}: variants may set overrides.<key>, "
                                "stop.sim_s, stop.settle_s or warmup_s")
            _set_dotted(body, key, value)
        try:
            variants.append((v["label"], _resolve_one(body)))
        except SpecError as e:
            raise SpecError(f"variant {v['label']!r}: {e}") from None

    return {"name": name, "title": str(raw.get("title") or name),
            "description": str(raw.get("description") or "").strip(),
            "repeats": reps, "pass_rate": pass_rate, "record": list(record),
            "variants": variants}


def context(resolved: dict) -> metrics_lib.Context:
    return metrics_lib.Context(
        robots=resolved["robots"], world=resolved["world"],
        footprint=robots_lib.footprint_points(), camera_hfov_rad=robots_lib.camera_hfov_rad(),
        goals=resolved["goals"], tasks=resolved["tasks"])


def streams_for(test: dict, resolved: dict) -> set[str]:
    """What the probe must subscribe to and record for one variant."""
    from .events import ALWAYS_RECORDED

    need = metrics_lib.MetricSet(resolved["metrics"], context(resolved)).streams()
    return set(ALWAYS_RECORDED) | need | set(test["record"])


# ---------------------------------------------------------------------------
# files
# ---------------------------------------------------------------------------

def path_for(name: str) -> Path:
    if not isinstance(name, str) or not NAME_RX.match(name):
        raise SpecError(f"test name {name!r} must match {NAME_RX.pattern}")
    return TESTS_DIR / f"{name}.yaml"


def load_raw(name: str) -> dict:
    p = path_for(name)
    try:
        return yaml.safe_load(p.read_text())
    except FileNotFoundError:
        raise SpecError(f"no test named {name!r} (looked for {p})") from None
    except yaml.YAMLError as e:
        raise SpecError(f"{p.name}: {e}") from None


def load(name: str, sets: list[tuple[str, object]] = ()) -> dict:
    raw = load_raw(name)
    for key, value in sets:
        _set_dotted(raw, key, value)
    t = validate(raw, stem=name)
    t["raw"] = raw
    return t


def list_tests() -> list[dict]:
    """Every configs/tests/*.yaml; invalid ones kept and explained."""
    out = []
    for p in sorted(TESTS_DIR.glob("*.yaml")):
        try:
            t = load(p.stem)
            base = t["variants"][0][1]
            out.append({
                "name": t["name"], "title": t["title"], "description": t["description"],
                "status": "ready", "scenario": base["scenario"],
                "world": base["run"].get("world"), "robots": base["robots"],
                "variants": [label for label, _ in t["variants"]],
                "repeats": t["repeats"], "runs": len(t["variants"]) * t["repeats"],
                "sim_s": base["stop"]["sim_s"],
                "metrics": [{"id": e["id"], "metric": e["metric"],
                             "criteria": [f"{v} {metrics_lib.OP_TEXT[op]} {th}"
                                          for v, op, th in e["pass"]]}
                            for e in base["metrics"]],
            })
        except SpecError as e:
            out.append({"name": p.stem, "title": p.stem, "description": "",
                        "status": "invalid", "error": str(e)})
    return out
