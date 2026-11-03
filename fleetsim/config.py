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
      rate_hz: 0.1
      pickups: "shelf_*"           # fnmatch over the world's waypoint names
      dropoffs: "dock_*"
      start_s: 0
      until_s: 300                 # default: duration_s
      count: 20                    # optional cap
    disturbances:
      - {at_s: 60, duration_s: 30, partition: [[robot_0, robot_1], rest]}
      - {at_s: 90, kill: robot_3}             # duration_s: revive after; fresh: true = reboot
      - {at_s: 30, duration_s: 60, block_link: [robot_2, gateway]}
      - {at_s: 40, blockage: {rect: [x0, y0, x1, y1]}}  # a static obstacle appears
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

FLEETSIM_DIR = Path(__file__).resolve().parent
BASE = FLEETSIM_DIR / "configs" / "base.yaml"
SCENARIOS_DIR = Path(os.environ.get("FLEETSIM_SCENARIOS_DIR") or FLEETSIM_DIR / "scenarios")
NAME_RX = re.compile(r"^[a-z0-9][a-z0-9_-]{0,63}$")
TOP_KEYS = {"name", "title", "description", "seed", "duration_s", "robots", "config", "tasks",
            "task_arrival", "disturbances", "stop", "metrics"}
# Blocks whose keys are not fixed by base.yaml.
FREE_FORM = {("world", "synthetic"), ("fleet_tasks",), ("network", "radio", "radio_model")}
DISTURBANCE_KINDS = {"kill", "revive", "partition", "block_link", "blockage"}


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
    }
    _validate(out)
    return out


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
    for key in ("status", "map_share"):
        if cfg[key]["recipients"] not in ("all", "radius"):
            raise ScenarioError(f"config.{key}.recipients: all | radius")
    if cfg["map_share"]["prior"] not in ("none", "truth"):
        raise ScenarioError("config.map_share.prior: none | truth")
    if cfg["tasks"]["bid_cost"] not in ("path", "straight"):
        raise ScenarioError("config.tasks.bid_cost: path | straight")
    if cfg["network"]["radio"]["contention"] not in ("global", "spatial"):
        raise ScenarioError("config.network.radio.contention: global | spatial")
    for i, t in enumerate(s["tasks"]):
        if not isinstance(t, dict) or {"at_s", "pickup", "dropoff"} - set(t):
            raise ScenarioError(f"tasks[{i}]: need at_s, pickup, dropoff")
    arr = s["task_arrival"]
    if arr is not None and (not isinstance(arr, dict) or not arr.get("rate_hz")
                            or float(arr["rate_hz"]) <= 0):
        raise ScenarioError("task_arrival: needs rate_hz > 0")
    for i, d in enumerate(s["disturbances"]):
        if not isinstance(d, dict) or "at_s" not in d or len(DISTURBANCE_KINDS & set(d)) != 1:
            raise ScenarioError(f"disturbances[{i}]: need at_s and exactly one of "
                                f"{sorted(DISTURBANCE_KINDS)}")
    until = s["stop"].get("until")
    if until not in (None, "tasks_done"):
        raise ScenarioError("stop.until: tasks_done (or omit to run for duration_s)")
    if s["metrics"]:
        try:
            s["metrics_resolved"] = metrics_lib.validate_list(s["metrics"])
        except metrics_lib.MetricError as e:
            raise ScenarioError(f"metrics: {e}") from None


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
