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
    tasks:                         # fixed deliveries (waypoint names)
      - {at_s: 5, pickup: shelf_0_0, dropoff: dock_0}
    task_arrival:                  # and/or a Poisson stream
      rate_hz: 0.1                 # or rate_per_robot_hz: x robots (one of the two)
      pickups: "shelf_*"           # fnmatch over the world's waypoint names
      dropoffs: "dock_*"
      start_s: 0
      until_s: 300                 # default: duration_s
      count: 20                    # optional cap; or count_per_robot: x robots
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
    for i, t in enumerate(s["tasks"]):
        if not isinstance(t, dict) or {"at_s", "pickup", "dropoff"} - set(t):
            raise ScenarioError(f"tasks[{i}]: need at_s, pickup, dropoff")
    arr = s["task_arrival"]
    if arr is not None:
        # rate_per_robot_hz keeps one load regime when a benchmark sweeps
        # --robots; rate_hz is a fixed stream whatever the fleet size.
        rates = [k for k in ("rate_hz", "rate_per_robot_hz") if isinstance(arr, dict) and k in arr]
        if len(rates) != 1 or not _num(arr[rates[0]]) or float(arr[rates[0]]) <= 0:
            raise ScenarioError("task_arrival: needs exactly one of rate_hz, rate_per_robot_hz, > 0")
        if "count" in arr and "count_per_robot" in arr:
            raise ScenarioError("task_arrival: count or count_per_robot, not both")
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
