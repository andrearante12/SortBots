"""The metric registry: how a test's performance is calculated.

A metric is a small class that reads events (simtest/events.py) and returns a
flat dict of named values. A test case lists the metrics it wants, with
parameters and pass criteria per value:

    metrics:
      - metric: clearance
        params: {near_m: 0.5}
        pass: {min_m: {gt: 0.0}, contacts: {eq: 0}}

Adding a metric — the whole contract:

    @register
    class MyMetric(Metric):
        name = "my_metric"          # what test cases call it by
        version = 1                 # bump whenever the CALCULATION changes
        streams = {"gt"}            # simtest.events.STREAMS this reads
        summary = "one line for the catalog"
        params = {"near_m": (float, 0.5, "what it means")}
        values = {"min_m": "what it is, with units"}

        def on_event(self, ev): ...  # only events from `streams`
        def result(self) -> dict:    # {value_name: number | bool | str | None}
            ...

then import its module at the bottom of this file. Changing how an existing
metric is calculated is the same edit plus a version bump: every result
records the version it was scored with, reports flag a comparison across
versions, and `scripts/simtest.py rescore` re-scores recorded runs with the
current code. That is the point of keeping metrics off ROS — see
simtest/events.py.

Metrics must be deterministic functions of their events (no wall clock, no
randomness), or rescoring stops meaning anything.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

REGISTRY: dict[str, type["Metric"]] = {}

OPS = {
    "eq": lambda v, x: v == x,
    "ne": lambda v, x: v != x,
    "gt": lambda v, x: v > x,
    "gte": lambda v, x: v >= x,
    "lt": lambda v, x: v < x,
    "lte": lambda v, x: v <= x,
}
OP_TEXT = {"eq": "==", "ne": "!=", "gt": ">", "gte": ">=", "lt": "<", "lte": "<="}


class MetricError(ValueError):
    pass


@dataclass
class Context:
    """What a metric knows besides its events. Built from the run's resolved
    test spec, so a rescore sees exactly what the live run saw."""

    robots: list[str]
    world: dict | None = None            # simtest.world.validate() output
    footprint: tuple = ()                # Nav2 footprint, body frame
    camera_hfov_rad: float = math.radians(87.0)
    goals: list[dict] = field(default_factory=list)   # the test's resolved `goals:`
    tasks: list[dict] = field(default_factory=list)   # the test's `tasks:`

    def actors(self, ids=None) -> list[dict]:
        acts = (self.world or {}).get("actors", [])
        return [a for a in acts if ids is None or a["id"] in ids]

    def props(self) -> list[dict]:
        return (self.world or {}).get("props", [])


class Metric:
    name: str = ""
    version: int = 1
    streams: frozenset | set = frozenset()
    summary: str = ""
    params: dict[str, tuple] = {}   # name -> (type, default, doc)
    values: dict[str, str] = {}     # name -> doc

    def __init__(self, ctx: Context, **params):
        self.ctx = ctx
        self.p = self.resolve_params(params)

    @classmethod
    def resolve_params(cls, given: dict) -> dict:
        unknown = set(given) - set(cls.params)
        if unknown:
            raise MetricError(f"{cls.name}: unknown param(s) {sorted(unknown)} "
                              f"(has: {sorted(cls.params)})")
        out = {}
        for key, (typ, default, _doc) in cls.params.items():
            v = given.get(key, default)
            if v is not None:
                try:
                    if typ is float and not isinstance(v, bool):
                        v = float(v)
                    elif typ is int and not isinstance(v, bool):
                        v = int(v)
                    elif typ is list:
                        if not isinstance(v, list) or not all(isinstance(i, str) for i in v):
                            raise TypeError
                    elif not isinstance(v, typ):
                        raise TypeError
                except (TypeError, ValueError):
                    raise MetricError(f"{cls.name}.params.{key}: expected "
                                      f"{typ.__name__}, got {v!r}") from None
            out[key] = v
        return out

    def check_context(self) -> None:
        """Raise MetricError if this metric can't be computed for the test
        (e.g. an actor metric on a world with no actors). Called at
        validation time, so a mis-specified test fails before the sim boots."""

    def on_event(self, ev: dict) -> None:  # pragma: no cover - interface
        raise NotImplementedError

    def result(self) -> dict:  # pragma: no cover - interface
        raise NotImplementedError

    def detail(self) -> dict:
        """Optional unscored breakdown (per-pair, per-goal, ...) for the report."""
        return {}


def register(cls):
    if not cls.name or cls.name in REGISTRY:
        raise RuntimeError(f"metric name {cls.name!r} is empty or registered twice")
    undocumented = [k for k in cls.values if not cls.values[k]]
    if undocumented:
        raise RuntimeError(f"{cls.name}: document every value ({undocumented})")
    REGISTRY[cls.name] = cls
    return cls


def get(name: str) -> type[Metric]:
    try:
        return REGISTRY[name]
    except KeyError:
        raise MetricError(f"no metric named {name!r} (have: {sorted(REGISTRY)})") from None


def catalog() -> list[dict]:
    return [{"name": c.name, "version": c.version, "summary": c.summary,
             "streams": sorted(c.streams),
             "params": {k: {"type": t.__name__, "default": d, "doc": doc}
                        for k, (t, d, doc) in c.params.items()},
             "values": dict(c.values)}
            for c in sorted(REGISTRY.values(), key=lambda c: c.name)]


# ---------------------------------------------------------------------------
# a test case's `metrics:` list
# ---------------------------------------------------------------------------

def validate_entry(entry, index: int) -> dict:
    """One `metrics:` item -> {id, metric, params, pass: [(value, op, threshold)]}."""
    w = f"metrics[{index}]"
    if not isinstance(entry, dict):
        raise MetricError(f"{w}: expected a mapping")
    unknown = set(entry) - {"metric", "id", "params", "pass"}
    if unknown:
        raise MetricError(f"{w}: unknown key(s) {sorted(unknown)}")
    cls = get(entry.get("metric"))
    params = cls.resolve_params(entry.get("params") or {})
    mid = entry.get("id") or cls.name
    if not isinstance(mid, str) or not mid.replace("_", "").isalnum():
        raise MetricError(f"{w}.id: must be a bare identifier")
    criteria = []
    raw_pass = entry.get("pass") or {}
    if not isinstance(raw_pass, dict):
        raise MetricError(f"{w}.pass: expected {{value: {{op: threshold}}}}")
    for value, cond in raw_pass.items():
        if value not in cls.values:
            raise MetricError(f"{w}.pass.{value}: {cls.name} has no value {value!r} "
                              f"(has: {sorted(cls.values)})")
        if not isinstance(cond, dict) or not cond:
            raise MetricError(f"{w}.pass.{value}: expected {{op: threshold}}, ops {sorted(OPS)}")
        for op, threshold in cond.items():
            if op not in OPS:
                raise MetricError(f"{w}.pass.{value}.{op}: unknown op (have {sorted(OPS)})")
            if op not in ("eq", "ne") and (isinstance(threshold, bool)
                                            or not isinstance(threshold, (int, float))):
                raise MetricError(f"{w}.pass.{value}.{op}: threshold must be a number")
            criteria.append((value, op, threshold))
    return {"id": mid, "metric": cls.name, "params": params, "pass": criteria}


def validate_list(entries) -> list[dict]:
    if not isinstance(entries, list) or not entries:
        raise MetricError("metrics: need at least one metric")
    out = [validate_entry(e, i) for i, e in enumerate(entries)]
    ids = [e["id"] for e in out]
    dupes = sorted({i for i in ids if ids.count(i) > 1})
    if dupes:
        raise MetricError(f"metrics: duplicate id(s) {dupes} — give repeats an `id:`")
    return out


def judge(value, op: str, threshold) -> bool | None:
    """None when there is nothing to judge (metric had no data)."""
    if value is None or (isinstance(value, float) and math.isnan(value)):
        return None
    try:
        return bool(OPS[op](value, threshold))
    except TypeError:
        return None


class MetricSet:
    """Instantiates a test's metrics and drives events through them."""

    def __init__(self, entries: list[dict], ctx: Context):
        self.entries = entries
        self.ctx = ctx
        self.instances = []
        for e in entries:
            m = get(e["metric"])(ctx, **e["params"])
            m.check_context()
            self.instances.append(m)
        self.by_stream: dict[str, list[Metric]] = {}
        for m in self.instances:
            for s in m.streams:
                self.by_stream.setdefault(s, []).append(m)

    def streams(self) -> set[str]:
        return set(self.by_stream)

    def feed(self, ev: dict) -> None:
        for m in self.by_stream.get(ev["s"], ()):
            m.on_event(ev)

    def score(self) -> dict:
        """{ok, metrics: [{id, metric, version, params, values, detail, criteria, ok}]}.

        A criterion with no data (value None) FAILS: a clearance metric that
        never saw the actor, or a detection metric that never had it in view,
        is a test that didn't test what it says — not a pass.
        """
        out, all_ok = [], True
        for e, m in zip(self.entries, self.instances):
            vals = m.result()
            crit = []
            for value, op, threshold in e["pass"]:
                ok = judge(vals.get(value), op, threshold)
                crit.append({"value": value, "op": op, "threshold": threshold,
                             "actual": vals.get(value), "ok": bool(ok),
                             "no_data": ok is None})
            ok = all(c["ok"] for c in crit)
            all_ok &= ok
            out.append({"id": e["id"], "metric": e["metric"], "version": m.version,
                        "params": e["params"], "values": vals, "detail": m.detail(),
                        "criteria": crit, "ok": ok})
        return {"ok": all_ok, "metrics": out}


# Registration: importing a module registers its metrics. Keep this list the
# single place a new metric module is wired in.
from . import run, clearance, goals, contamination, detection, coverage, tasks  # noqa: E402,F401
