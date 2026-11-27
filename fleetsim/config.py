"""Scenarios: fleetsim/scenarios/<name>.yaml — one fleet, one question.

    name: delivery_smoke
    title: Eight robots, a dozen deliveries, ideal network
    description: >
      ...
    seed: 1
    duration_s: 600
    robots: 8                      # shorthand for config.robots.count
    config:                        # layered over fleetsim/configs/base.yaml
      network: {model: radio}
      robots:                      # robot types (base.yaml robots.types); null = one type
        types:
          - {name: standard}                                   # the rest
          - {name: heavy, share: 0.2, capabilities: [heavy], pick_s: 6}
    tasks:                         # fixed deliveries (waypoint names)
      - {at_s: 5, pickup: shelf_0_0, dropoff: dock_0}
      - {at_s: 9, pickup: shelf_1_0, dropoff: dock_1, requires: heavy}  # or a list
    task_arrival:                  # and/or a Poisson stream
      rate_hz: 0.1                 # or rate_per_robot_hz: x robots (one of the two)
      pickups: "shelf_*"           # fnmatch over the world's waypoint names
      dropoffs: "dock_*"
      start_s: 0
      until_s: 300                 # default: duration_s
      count: 20                    # optional cap; or count_per_robot: x robots
      requires: {heavy: 0.2}       # share of arrivals needing each capability (one each)
    disturbances:
      - {at_s: 60, duration_s: 30, partition: [[robot_0, robot_1], rest]}
      - {at_s: 60, duration_s: 30, partition: [{first: 0.4}, rest]}  # 40 % of any fleet
      - {at_s: 90, kill: robot_3}             # duration_s: revive after; fresh: true = reboot
      - {at_s: 90, kill: {select: carrying}}  # whoever holds an item then (fetching, busy, any;
                                    # within_s: 300 = wait up to 300 s for one)
      - {at_s: 90, duration_s: 60, kill: gateway}  # requests wait for it (never fresh)
      - {at_s: 30, duration_s: 60, block_link: [robot_2, gateway]}
      - {at_s: 40, blockage: {rect: [x0, y0, x1, y1]}}  # a static obstacle appears
                                    # (once no robot is under it; duration_s: it clears)
    stop: {until: tasks_done, settle_s: 5}  # default: run for duration_s
    metrics:                       # simtest metrics (simtest/metrics) with pass criteria
      - metric: fleet_tasks
        pass: {done_frac: {gte: 1.0}}

`--set a.b=v` on the command line overrides the resolved config (a YAML
value), and `--robots`, `--seed`, `--duration` the top-level shorthands.
"""
from __future__ import annotations

import copy
import math
import os
import re
from pathlib import Path

import yaml

from simtest import metrics as metrics_lib
from simtest import world as world_lib

FLEETSIM_DIR = Path(__file__).resolve().parent
BASE = FLEETSIM_DIR / "configs" / "base.yaml"
SCENARIOS_DIR = Path(os.environ.get("FLEETSIM_SCENARIOS_DIR") or FLEETSIM_DIR / "scenarios")
# Where `fleetsim run` writes and the replay view reads (gitignored). The
# override is for tests: webui/tests/operator_test.mjs replays a run it makes.
RESULTS_DIR = Path(os.environ.get("FLEETSIM_RESULTS_DIR") or FLEETSIM_DIR.parent / "data" / "fleetsim")
NAME_RX = re.compile(r"^[a-z0-9][a-z0-9_-]{0,63}$")
TOP_KEYS = {"name", "title", "description", "seed", "duration_s", "robots", "config", "tasks",
            "task_arrival", "disturbances", "stop", "metrics", "actors"}
# Blocks whose keys are not fixed by base.yaml.
FREE_FORM = {("world", "synthetic"), ("fleet_tasks",), ("network", "radio", "radio_model")}
DISTURBANCE_KINDS = {"kill", "revive", "partition", "block_link", "blockage"}
DISPATCH_METHODS = ("auction", "central_greedy", "central_hungarian")  # fleetsim/dispatch
INFO_CHOICES = {"map": ("shared", "truth"), "peers": ("heard", "truth"),
                "dispatch": ("network", "truth")}
# kill: {select: ...} picks its robot when it fires, lowest index first.
KILL_SELECT = ("carrying", "fetching", "busy", "any")
ROBOT_RX = re.compile(r"^robot_(\d+)$")
# Robot types (A6; docs/fleetsim.md "Robot types and capabilities").
TYPE_KEYS = {"name", "share", "count", "max_speed_mps", "accel_mps2", "yaw_rate_rps",
             "pick_s", "place_s", "battery_drain_pct_per_s", "capabilities"}
CAP_RX = re.compile(r"^[a-z0-9][a-z0-9_-]{0,31}$")


class ScenarioError(ValueError):
    pass


def load_base() -> dict:
    return yaml.safe_load(BASE.read_text())


def _free(path: tuple) -> bool:
    return any(path[:len(f)] == f for f in FREE_FORM)


def merge_checked(base: dict, over: dict, path: tuple = ()) -> dict:
    out = copy.deepcopy(base)
    for k, v in (over or {}).items():
        here = path + (k,)
        if _free(path):  # inside a free-form block: anything goes
            # ...but merged, not replaced: two `--set fleet_tasks.protocol.*`
            # kept only the last, because the second replaced the whole
            # protocol mapping the first had made (2026-11-17).
            if isinstance(out.get(k), dict) and isinstance(v, dict):
                out[k] = merge_checked(out[k], v, here)
            else:
                out[k] = copy.deepcopy(v)
            continue
        if k not in out:
            raise ScenarioError(f"config.{'.'.join(here)}: no such setting "
                                f"(fleetsim/configs/base.yaml has: {sorted(out)})")
        if isinstance(out[k], dict) and isinstance(v, dict):
            out[k] = merge_checked(out[k], v, here)
        elif isinstance(out[k], dict):
            raise ScenarioError(f"config.{'.'.join(here)}: expected a mapping")
        else:
            out[k] = copy.deepcopy(v)
    return out


def _set_dotted(d: dict, dotted: str, value) -> dict:
    keys = dotted.split(".")
    over: dict = {}
    cur = over
    for k in keys[:-1]:
        cur = cur.setdefault(k, {})
    cur[keys[-1]] = value
    return merge_checked(d, over)


def resolve(raw: dict, *, sets: list[tuple[str, object]] = (), robots: int | None = None,
            seed: int | None = None, duration: float | None = None) -> dict:
    """A scenario mapping -> a fully resolved, validated scenario dict."""
    if not isinstance(raw, dict):
        raise ScenarioError("a scenario must be a YAML mapping")
    unknown = set(raw) - TOP_KEYS
    if unknown:
        raise ScenarioError(f"unknown key(s) {sorted(unknown)} (have: {sorted(TOP_KEYS)})")
    name = raw.get("name")
    if not isinstance(name, str) or not NAME_RX.match(name):
        raise ScenarioError(f"name: must match {NAME_RX.pattern}")
    cfg = merge_checked(load_base(), raw.get("config") or {})
    n = robots if robots is not None else raw.get("robots")
    if n is not None:
        cfg["robots"]["count"] = n
    for k, v in sets:
        cfg = _set_dotted(cfg, k, v)
    out = {
        "name": name,
        "title": str(raw.get("title") or name),
        "description": str(raw.get("description") or "").strip(),
        "seed": int(seed if seed is not None else raw.get("seed", 1)),
        "duration_s": float(duration if duration is not None else raw.get("duration_s", 600)),
        "config": cfg,
        "tasks": list(raw.get("tasks") or []),
        "task_arrival": raw.get("task_arrival"),
        "disturbances": list(raw.get("disturbances") or []),
        "stop": raw.get("stop") or {},
        "metrics": raw.get("metrics") or [],
        "actors": _actors(name, raw.get("actors"), cfg),
    }
    _validate(out)
    return out


def _actors(name: str, inline, cfg: dict) -> list:
    """Scripted movers: inline `actors:` (simtest/world.py's actor schema —
    for a synthetic floor, whose coordinates no configs/worlds file shares),
    or config.world.actors naming a configs/worlds world."""
    if inline and cfg["world"]["actors"]:
        raise ScenarioError("actors: give inline actors or config.world.actors, not both")
    try:
        if inline:
            return world_lib.validate({"name": name, "actors": inline})["actors"]
        if cfg["world"]["actors"]:
            return world_lib.load(cfg["world"]["actors"])["actors"]
    except world_lib.WorldError as e:
        raise ScenarioError(f"actors: {e}") from None
    return []


def _validate(s: dict) -> None:
    cfg = s["config"]
    n = cfg["robots"]["count"]
    if not isinstance(n, int) or n < 1 or n > 1000:
        raise ScenarioError(f"robots: expected 1..1000, got {n!r}")
    if not 0 < s["duration_s"] <= 86400:
        raise ScenarioError("duration_s: expected (0, 86400]")
    if cfg["world"]["source"] not in ("synthetic", "map"):
        raise ScenarioError("config.world.source: synthetic | map")
    if cfg["world"]["source"] == "map" and not cfg["world"]["map"]:
        raise ScenarioError("config.world.map: a saved map name is needed with source: map")
    if not 0.05 <= float(cfg["world"]["cell_m"]) <= 0.5:
        raise ScenarioError("config.world.cell_m: expected 0.05..0.5 m")
    if cfg["network"]["model"] not in ("ideal", "radio"):
        raise ScenarioError("config.network.model: ideal | radio")
    if cfg["intent"]["transport"] not in ("status", "message"):
        raise ScenarioError("config.intent.transport: status | message")
    if cfg["network"]["capacity"] not in ("enforced", "unlimited"):
        raise ScenarioError("config.network.capacity: enforced | unlimited")
    for key in ("status", "map_share"):
        if cfg[key]["recipients"] not in ("all", "radius"):
            raise ScenarioError(f"config.{key}.recipients: all | radius")
    if cfg["map_share"]["prior"] not in ("none", "truth"):
        raise ScenarioError("config.map_share.prior: none | truth")
    if cfg["tasks"]["bid_cost"] not in ("path", "straight"):
        raise ScenarioError("config.tasks.bid_cost: path | straight")
    if cfg["network"]["radio"]["contention"] not in ("global", "spatial"):
        raise ScenarioError("config.network.radio.contention: global | spatial")
    dsp = cfg["dispatch"]
    if dsp["method"] not in DISPATCH_METHODS:
        raise ScenarioError(f"config.dispatch.method: {' | '.join(DISPATCH_METHODS)}")
    if not isinstance(dsp["reassign_gain_s"], (int, float)) or dsp["reassign_gain_s"] < 0:
        raise ScenarioError("config.dispatch.reassign_gain_s: a number >= 0")
    if not isinstance(dsp["resolve_period_s"], (int, float)) or dsp["resolve_period_s"] <= 0:
        raise ScenarioError("config.dispatch.resolve_period_s: a number > 0")
    for key, ok in INFO_CHOICES.items():
        if cfg["info"][key] not in ok:
            raise ScenarioError(f"config.info.{key}: {' | '.join(ok)}")
    caps = _check_types(cfg)
    for i, t in enumerate(s["tasks"]):
        if not isinstance(t, dict) or {"at_s", "pickup", "dropoff"} - set(t):
            raise ScenarioError(f"tasks[{i}]: need at_s, pickup, dropoff")
        if "requires" in t:
            _check_known(requirement(t["requires"], f"tasks[{i}].requires"), caps,
                         f"tasks[{i}].requires")
    arr = s["task_arrival"]
    if arr is not None:
        # rate_per_robot_hz keeps one load regime when a benchmark sweeps
        # --robots; rate_hz is a fixed stream whatever the fleet size.
        rates = [k for k in ("rate_hz", "rate_per_robot_hz") if isinstance(arr, dict) and k in arr]
        if len(rates) != 1 or not _num(arr[rates[0]]) or float(arr[rates[0]]) <= 0:
            raise ScenarioError("task_arrival: needs exactly one of rate_hz, rate_per_robot_hz, > 0")
        if "count" in arr and "count_per_robot" in arr:
            raise ScenarioError("task_arrival: count or count_per_robot, not both")
        if "requires" in arr:
            arrival_requires(arr["requires"], caps)
    for i, d in enumerate(s["disturbances"]):
        if not isinstance(d, dict) or "at_s" not in d or len(DISTURBANCE_KINDS & set(d)) != 1:
            raise ScenarioError(f"disturbances[{i}]: need at_s and exactly one of "
                                f"{sorted(DISTURBANCE_KINDS)}")
        _check_disturbance(d, n, f"disturbances[{i}]")
    if cfg["world"]["source"] == "synthetic":
        _check_floor(cfg)
    bm = cfg["record"]["belief_maps"]
    names = {f"robot_{k}" for k in range(n)}
    if bm != "all" and (not isinstance(bm, list) or set(bm) - names):
        raise ScenarioError(f"config.record.belief_maps: `all` or a list of robot_0..robot_{n - 1}")
    until = s["stop"].get("until")
    if until not in (None, "tasks_done"):
        raise ScenarioError("stop.until: tasks_done (or omit to run for duration_s)")
    if s["metrics"]:
        try:
            s["metrics_resolved"] = metrics_lib.validate_list(s["metrics"])
        except metrics_lib.MetricError as e:
            raise ScenarioError(f"metrics: {e}") from None


def _num(v) -> bool:
    return isinstance(v, (int, float)) and not isinstance(v, bool)


# ---- robot types and capabilities (A6) ----------------------------------------

def requirement(v, where: str = "requires") -> tuple:
    """A task's `requires`: one capability name or a list of them, as a
    sorted tuple (empty = any robot)."""
    if v is None:
        return ()
    if isinstance(v, str):
        v = [v]
    if not isinstance(v, list) or not all(isinstance(c, str) and CAP_RX.match(c) for c in v):
        raise ScenarioError(f"{where}: a capability name or a list of them "
                            f"(each matching {CAP_RX.pattern})")
    return tuple(sorted(set(v)))


def arrival_requires(v, caps: set | None = None) -> list[tuple[str, float]]:
    """task_arrival.requires: {capability: share of arrivals that need it}.
    One capability per task, so the shares add up to at most 1 and the rest
    need nothing; in the order given (each arrival draws once, sim.py)."""
    where = "task_arrival.requires"
    if not isinstance(v, dict) or not v:
        raise ScenarioError(f"{where}: {{capability: share, ...}}, e.g. {{heavy: 0.2}}")
    out = []
    for c, f in v.items():
        if not isinstance(c, str) or not CAP_RX.match(c):
            raise ScenarioError(f"{where}: {c!r} is not a capability name ({CAP_RX.pattern})")
        if not _num(f) or not 0 < f <= 1:
            raise ScenarioError(f"{where}.{c}: a share in (0, 1], got {f!r}")
        out.append((c, float(f)))
    if sum(f for _, f in out) > 1 + 1e-9:
        raise ScenarioError(f"{where}: the shares add up to more than 1 (one capability per task)")
    if caps is not None:
        _check_known(tuple(c for c, _ in out), caps, where)
    return out


def _check_known(req: tuple, caps: set, where: str) -> None:
    """A capability no robot type lists is a typo far more often than a test:
    fail it here. A type with no robots at this fleet size (`count: 0`, or
    the rest when the others take them all) passes, and its tasks are
    reported at run time instead (sim.py, `unservable`)."""
    missing = sorted(set(req) - caps)
    if missing:
        have = f"the types list {sorted(caps)}" if caps else "config.robots.types is not set"
        raise ScenarioError(f"{where}: no robot type has {missing} ({have})")


def _type_counts(types: list, n: int) -> list[int]:
    """Robots per type at fleet size n: `count` is fixed, `share` is
    round(share x n) and at least 1 (so a capability a 20-robot fleet has, a
    2-robot one has too), and the one type with neither takes the rest. With
    no such type the counts must come to n exactly."""
    counts, rest = [], None
    for j, t in enumerate(types):
        if "count" in t:
            counts.append(int(t["count"]))
        elif "share" in t:
            counts.append(max(1, int(math.floor(float(t["share"]) * n + 0.5))))
        else:
            counts.append(0)
            rest = j
    taken = sum(counts)
    names = [t["name"] for j, t in enumerate(types) if j != rest]
    if rest is not None:
        if taken > n:
            raise ScenarioError(f"config.robots.types: {', '.join(names)} take {taken} robots, "
                                f"the fleet has {n}")
        counts[rest] = n - taken
    elif taken != n:
        raise ScenarioError(f"config.robots.types: the types make {taken} robots at a fleet of "
                            f"{n}; leave one type without share or count to take the rest")
    return counts


def _check_types(cfg: dict) -> set:
    """Validate config.robots.types; the set of capabilities they list."""
    types = cfg["robots"]["types"]
    if types is None:
        return set()
    where = "config.robots.types"
    if not isinstance(types, list) or not types:
        raise ScenarioError(f"{where}: null, or a list of types ({{name: ..., share: ...}})")
    seen, caps, rest = set(), set(), 0
    for j, t in enumerate(types):
        w = f"{where}[{j}]"
        if not isinstance(t, dict):
            raise ScenarioError(f"{w}: a mapping")
        unknown = set(t) - TYPE_KEYS
        if unknown:
            raise ScenarioError(f"{w}: unknown key(s) {sorted(unknown)} (have: {sorted(TYPE_KEYS)})")
        name = t.get("name")
        if not isinstance(name, str) or not NAME_RX.match(name) or name in seen:
            raise ScenarioError(f"{w}.name: a unique name matching {NAME_RX.pattern}")
        seen.add(name)
        if "share" in t and "count" in t:
            raise ScenarioError(f"{w}: share or count, not both")
        if "share" in t and (not _num(t["share"]) or not 0 < t["share"] <= 1):
            raise ScenarioError(f"{w}.share: in (0, 1]")
        if "count" in t and (not isinstance(t["count"], int) or isinstance(t["count"], bool)
                             or t["count"] < 0):
            raise ScenarioError(f"{w}.count: an integer >= 0")
        rest += "share" not in t and "count" not in t
        for k, lo, hi in (("max_speed_mps", 0.0, 5.0), ("accel_mps2", 0.0, 10.0),
                          ("yaw_rate_rps", 0.0, 10.0)):
            if k in t and (not _num(t[k]) or not lo < t[k] <= hi):
                raise ScenarioError(f"{w}.{k}: in ({lo:g}, {hi:g}]")
        for k in ("pick_s", "place_s", "battery_drain_pct_per_s"):
            if k in t and (not _num(t[k]) or not 0 <= t[k] <= 3600):
                raise ScenarioError(f"{w}.{k}: in [0, 3600]")
        caps |= set(requirement(t.get("capabilities"), f"{w}.capabilities"))
    if rest > 1:
        raise ScenarioError(f"{where}: at most one type without share or count (it takes the rest)")
    _type_counts(types, cfg["robots"]["count"])
    return caps


def robot_profiles(cfg: dict) -> list[dict]:
    """What each robot of the fleet is, by index: its type's name, speed,
    acceleration, yaw rate, pick and place times, battery drain (None: the
    protocol's fleet-wide value) and capabilities. With robots.types null,
    every robot is the base robots.* values with tasks.dwell_s for both
    service times, exactly what fleetsim had before types.

    Types are spread through the fleet, not given in blocks: robot_k parks
    at the k-th home and wins exact auction ties by its index, so a block
    would park every heavy robot at one end of the apron and hand one type
    the tie-breaks. Each type's robots take evenly spaced slots ((i + 1/2) x
    n / count), and the slots in order are robot_0, robot_1, ... No RNG."""
    rc = cfg["robots"]
    n = int(rc["count"])
    dwell = float(cfg["tasks"]["dwell_s"])
    base = {"type": "default", "max_speed_mps": float(rc["max_speed_mps"]),
            "accel_mps2": float(rc["accel_mps2"]), "yaw_rate_rps": float(rc["yaw_rate_rps"]),
            "pick_s": dwell, "place_s": dwell, "battery_drain_pct_per_s": None,
            "capabilities": ()}
    types = rc.get("types")
    if types is None:
        return [dict(base) for _ in range(n)]
    profiles = []
    for t in types:
        p = dict(base, type=t["name"], capabilities=requirement(t.get("capabilities")))
        for k in ("max_speed_mps", "accel_mps2", "yaw_rate_rps", "pick_s", "place_s",
                  "battery_drain_pct_per_s"):
            if t.get(k) is not None:
                p[k] = float(t[k])
        profiles.append(p)
    counts = _type_counts(types, n)
    slots = sorted(((i + 0.5) * n / c, j) for j, c in enumerate(counts) for i in range(c))
    return [dict(profiles[j]) for _, j in slots]


def _check_node(name, n: int, where: str) -> None:
    """A node name that exists in a fleet of n. A scenario run with --robots 2
    that names robot_3 used to die mid-run on an IndexError, and `kill:
    gateway` silently killed the last robot (node 0 - 1 = robots[-1])."""
    if name == "gateway":
        return
    m = ROBOT_RX.match(name) if isinstance(name, str) else None
    if m is None:
        raise ScenarioError(f"{where}: {name!r} is not a node (robot_<k> or gateway)")
    if int(m.group(1)) >= n:
        raise ScenarioError(f"{where}: no {name} in a fleet of {n} (robot_0..robot_{n - 1}); "
                            "a group like {first: 0.4} or kill: {select: busy} scales with "
                            "--robots")


def _check_group(g, n: int, where: str) -> None:
    if g == "rest":
        return
    for m in (g if isinstance(g, list) else [g]):
        if isinstance(m, dict):
            if len(m) != 1 or next(iter(m)) not in ("first", "last") \
                    or not _num(next(iter(m.values()))) or not 0 < next(iter(m.values())) < 1:
                raise ScenarioError(f"{where}: a fraction group is {{first: F}} or {{last: F}}, "
                                    f"0 < F < 1, got {m!r}")
        else:
            _check_node(m, n, where)


def _check_disturbance(d: dict, n: int, where: str) -> None:
    if "kill" in d:
        k = d["kill"]
        if isinstance(k, dict):
            if "select" not in k or set(k) - {"select", "within_s"} or k["select"] not in KILL_SELECT \
                    or not (_num(k.get("within_s", 0)) and k.get("within_s", 0) >= 0):
                raise ScenarioError(f"{where}: kill: {{select: {' | '.join(KILL_SELECT)}, "
                                    "within_s: S (optional, >= 0)}}")
        else:
            _check_node(k, n, f"{where}.kill")
            if k == "gateway" and d.get("fresh"):
                # A rebooted gateway would forget its board and restart task
                # ids at 0 (fleetsim's fixed clock), which robots would take
                # for tasks they already know.
                raise ScenarioError(f"{where}: a fresh gateway restart is not modelled")
    elif "revive" in d:
        _check_node(d["revive"], n, f"{where}.revive")
        if d["revive"] == "gateway" and d.get("fresh"):
            raise ScenarioError(f"{where}: a fresh gateway restart is not modelled")
    elif "block_link" in d:
        bl = d["block_link"]
        if not isinstance(bl, list) or len(bl) != 2:
            raise ScenarioError(f"{where}: block_link: [node, node]")
        for x in bl:
            _check_node(x, n, f"{where}.block_link")
    elif "partition" in d:
        groups = d["partition"]
        if not isinstance(groups, list) or len(groups) < 2 or groups.count("rest") > 1:
            raise ScenarioError(f"{where}: partition: two or more groups, at most one `rest`")
        for g in groups:
            _check_group(g, n, f"{where}.partition")
    elif "blockage" in d:
        b = d["blockage"]
        rect = b.get("rect") if isinstance(b, dict) else None
        if not isinstance(rect, list) or len(rect) != 4 or not all(_num(v) for v in rect):
            raise ScenarioError(f"{where}: blockage: {{rect: [x0, y0, x1, y1]}}")


def _check_floor(cfg: dict) -> None:
    """Build the synthetic floor and check every waypoint can be driven to,
    so a closed-off shelf fails `fleetsim list` and the scenario editor's
    save, not a run."""
    try:
        from . import grid as G  # numpy: serve.py's python may not have it
    except ImportError:
        return
    w = cfg["world"]
    cell = float(w["cell_m"])
    try:
        G.synthetic_checked(w.get("synthetic") or {}, cell,
                            clearance_m=float(cfg["robots"]["footprint_radius_m"]) + 0.5 * cell,
                            tol_m=float(cfg["nav"]["goal_tolerance_m"]))
    except (ValueError, TypeError, KeyError) as e:
        raise ScenarioError(f"config.{e}" if str(e).startswith("world.") else
                            f"config.world.synthetic: {e}") from None


def path_for(name: str) -> Path:
    if not NAME_RX.match(name or ""):
        raise ScenarioError(f"scenario name {name!r} must match {NAME_RX.pattern}")
    return SCENARIOS_DIR / f"{name}.yaml"


def load(name: str, **kw) -> dict:
    p = path_for(name)
    try:
        raw = yaml.safe_load(p.read_text())
    except FileNotFoundError:
        raise ScenarioError(f"no scenario {name!r} (looked for {p})") from None
    except yaml.YAMLError as e:
        raise ScenarioError(f"{p.name}: {e}") from None
    if isinstance(raw, dict) and raw.get("name") != p.stem:
        raise ScenarioError(f"{p.name}: name must equal the file stem {p.stem!r}")
    return resolve(raw, **kw)


def list_scenarios() -> list[dict]:
    out = []
    for p in sorted(SCENARIOS_DIR.glob("*.yaml")):
        try:
            s = load(p.stem)
            out.append({"name": s["name"], "title": s["title"], "robots": s["config"]["robots"]["count"],
                        "network": s["config"]["network"]["model"], "status": "ready"})
        except ScenarioError as e:
            out.append({"name": p.stem, "title": p.stem, "status": "invalid", "error": str(e)})
    return out
