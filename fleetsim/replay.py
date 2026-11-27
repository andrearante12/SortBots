"""A recorded fleetsim run, packed for the operator view's replay
(webui/operator_replay.js, served by webui/serve.py /api/fleetsim/*).

The browser gets ONE JSON document per run and scrubs it locally, so the
packing is about size: poses are flat arrays per sample (a 50-robot, 15-min
run at 5 Hz is ~225k poses, ~3 MB before the transfer's gzip) instead of
simtest's {rid: {x, y, yaw}} dicts, and everything else is a list of
[t, ...] rows in time order, so the page finds "the state at t" by bisection
instead of replaying from the start.

Nothing here simulates: it reads events.jsonl.gz and the JSON files beside
it. A run recorded before the `world` stream existed (2026-11-03) has no
floor to draw, so it is listed but not replayable; re-running its scenario
with the same seed reproduces it exactly.

A run is named by a REFERENCE, never a path: `<run>` for data/fleetsim/<run>/,
or `bench/<bench>/<run>` for one run of a `fleetsim bench` (bench.py writes
data/fleetsim/bench/<UTC>_<suite>/<run>/). Every part is matched by DIR_RX,
and the directory and each file read must resolve inside RESULTS_DIR, so a
symlink can't lead out of it (serve.py serves this without --control).

`comms` is what the replay's comms monitor draws: per net sample, the
fleet's load and traffic by kind, and (since 2026-11-26, record.net_nodes)
each node's load and traffic. A run recorded before that still packs: the
node rows are null and the monitor draws only the fleet.

Pure stdlib + simtest.events (no numpy: the grids stay the zlib+base64 JSON
they were recorded as, and the page inflates them).
"""
from __future__ import annotations

import gzip
import json
import re
from pathlib import Path

from simtest import events as ev_lib

from .config import RESULTS_DIR

# fleetsim/network.py LOAD_CAP; not imported, so this module stays numpy-free
# (serve.py loads it). fleetsim/tests/test_replay.py checks the two agree.
LOAD_CAP = 0.95

# A run directory NAME (never a path): what `fleetsim run` writes, or --out's
# basename. No `/` and no leading dot, so it can't leave RESULTS_DIR.
DIR_RX = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]{0,127}$")
BENCH = "bench"             # RESULTS_DIR/bench/<bench>/<run>/ (fleetsim/bench.py)
STAMP_RX = re.compile(r"^\d{8}T\d{6}Z_")  # bench.py's <UTC>_ prefix on a bench dir
# The monitor's kinds, in the order the page gives them colours; anything
# else follows, sorted. Colour follows the kind, never its rank in a run.
KIND_ORDER = ("status", "tasks", "map", "intent", "obstacle", "resync")
# Per kind in a `comms` row, after [t, dt, load_mean, load_max, over_cap, ogm_air].
KIND_FIELDS = ("sent", "delivered", "lost", "bytes", "msgs", "air_ms", "p95_ms")
# Per kind per node in a `comms` node row (network.py take_nodes).
NODE_FIELDS = ("sent", "bytes", "lost", "rx", "rx_bytes")
# Local-layer events worth a marker on the timeline; replans are far too
# frequent to mark one by one (hundreds a minute at 50 robots).
NAV_MARKS = {"blocked", "gave_up", "plan_failed", "empty_shelf", "recovery"}
TERMINAL = {"DONE", "FAILED", "CANCELLED", "REJECTED"}

_cache: dict = {}  # run reference -> (mtime, packed); a handful of runs at most
_CACHE_N = 4


class ReplayError(ValueError):
    pass


def _parts(ref) -> list[str] | None:
    parts = ref.split("/") if isinstance(ref, str) else []
    if len(parts) == 1 and DIR_RX.match(parts[0]) and parts[0] != BENCH:
        return parts
    if len(parts) == 3 and parts[0] == BENCH and all(DIR_RX.match(x) for x in parts[1:]):
        return parts
    return None


def valid_ref(ref) -> bool:
    """`<run>` or `bench/<bench>/<run>`, each part a DIR_RX name."""
    return _parts(ref) is not None


def _inside(root: Path, p: Path) -> bool:
    """p, symlinks followed, is under root (itself resolved: data/ may be a
    link to a bigger disk). A link that stays inside is fine, e.g. a bench
    run linked into data/fleetsim/ to list it with the others."""
    try:
        return p.resolve().is_relative_to(root.resolve())
    except (OSError, RuntimeError):  # a link loop
        return False


def run_dir(name: str, root: Path | None = None) -> Path:
    root = Path(root or RESULTS_DIR)
    parts = _parts(name)
    if parts is None:
        raise ReplayError("dir must be a fleetsim run directory name, or bench/<bench>/<run>")
    d = root.joinpath(*parts)
    ev = d / "events.jsonl.gz"
    if not (ev.is_file() and _inside(root, d) and _inside(root, ev)):
        raise FileNotFoundError(f"no fleetsim run {name!r}")  # never the path: serve.py is public
    return d


def _json(p: Path, root: Path | None = None):
    if root is not None and not _inside(root, p):
        return None
    try:
        return json.loads(p.read_text())
    except (OSError, ValueError):
        return None


def _has_world(events: Path) -> bool:
    """`world` comes straight after `start` (fleet_log lines from robot
    start-up can precede it); the rest of a multi-megabyte file needn't be
    inflated to know."""
    try:
        with gzip.open(events, "rt", encoding="utf-8") as fh:
            for _ in range(64):
                line = fh.readline()
                if not line:
                    return False
                if line.startswith('{"s":"world"'):
                    return True
    except (OSError, EOFError):
        return False
    return False


def _run_dirs(parent: Path, root: Path) -> list[Path]:
    """Run dirs directly under parent: DIR_RX names with an events file,
    inside root once links are followed."""
    if not parent.is_dir() or not _inside(root, parent):
        return []
    return [d for d in parent.iterdir()
            if DIR_RX.match(d.name) and d.is_dir() and (d / "events.jsonl.gz").is_file()
            and _inside(root, d) and _inside(root, d / "events.jsonl.gz")]


def _row(d: Path, ref: str, root: Path) -> dict:
    scn = _json(d / "scenario.json", root) or {}
    summ = _json(d / "summary.json", root)
    sc = _json(d / "score.json", root)
    cfg = scn.get("config") or {}
    return {
        "dir": ref,
        "name": scn.get("name") or d.name,
        "title": scn.get("title") or "",
        "robots": (cfg.get("robots") or {}).get("count"),
        "network": (cfg.get("network") or {}).get("model"),
        "seed": scn.get("seed"),
        "sim_s": summ.get("sim_s") if summ else None,
        "reason": summ.get("reason") if summ else None,
        # No summary.json: still running, or it died (docs/fleetsim.md "Known issues").
        "finished": summ is not None,
        "verdict": None if sc is None else ("PASS" if sc.get("ok") else "FAIL"),
        "replayable": _has_world(d / "events.jsonl.gz"),
        "mtime": round((d / "events.jsonl.gz").stat().st_mtime, 1),
    }


def list_runs(root: Path | None = None, limit: int = 50) -> list[dict]:
    root = Path(root or RESULTS_DIR)
    dirs = sorted(_run_dirs(root, root), key=lambda d: (d / "events.jsonl.gz").stat().st_mtime,
                  reverse=True)
    return [_row(d, d.name, root) for d in dirs[:limit]]


_bench_cache: dict = {}  # summary.json path -> (mtime, what the listing reads of it)


def _bench_summary(bdir: Path, root: Path) -> dict:
    """What the listing needs from a bench's summary.json, parsed once per
    change: the file holds every run's metric values (~1 MB for a few hundred
    runs) and the Tests tab lists benches every 10 s while it is open."""
    p = bdir / "summary.json"
    try:
        mtime = p.stat().st_mtime
    except OSError:
        return {}
    hit = _bench_cache.get(p)
    if hit and hit[0] == mtime:
        return hit[1]
    got = _json(p, root)
    got = got if isinstance(got, dict) else {}
    suite = got.get("suite") if isinstance(got.get("suite"), dict) else {}
    keep = {"suite": {k: suite.get(k) for k in ("name", "description")},
            "started": got.get("started"), "complete": got.get("complete"),
            "counts": got.get("counts") if isinstance(got.get("counts"), dict) else {},
            "runs": [{k: r.get(k) for k in ("name", "variant", "status")}
                     for r in got.get("runs") or [] if isinstance(r, dict)]}
    _bench_cache[p] = (mtime, keep)
    return keep


def list_benches(root: Path | None = None, limit: int = 30) -> list[dict]:
    """The benches under RESULTS_DIR/bench/, newest first, each with its run
    count but not its runs: a suite can be hundreds (list_bench has them)."""
    root = Path(root or RESULTS_DIR)
    parent = root / BENCH
    if not parent.is_dir() or not _inside(root, parent):
        return []
    out = []
    for b in parent.iterdir():
        if not (DIR_RX.match(b.name) and b.is_dir() and _inside(root, b)):
            continue
        runs = _run_dirs(b, root)
        if not runs:
            continue
        summ = _bench_summary(b, root)
        suite = summ.get("suite") or {}
        mtime = max((d / "events.jsonl.gz").stat().st_mtime for d in runs)
        out.append({"dir": b.name, "suite": suite.get("name") or STAMP_RX.sub("", b.name),
                    "description": suite.get("description") or "",
                    "started": summ.get("started"), "complete": bool(summ.get("complete")),
                    "counts": summ.get("counts") or {}, "runs": len(runs),
                    "mtime": round(mtime, 1)})
    out.sort(key=lambda r: r["mtime"], reverse=True)
    return out[:limit]


def list_bench(name: str, root: Path | None = None) -> list[dict]:
    """One bench's runs, in the order the bench planned them (scenario,
    robots, variant, seed), each with its bench status and variant."""
    root = Path(root or RESULTS_DIR)
    if not isinstance(name, str) or not DIR_RX.match(name):
        raise ReplayError("bench must be a fleetsim bench directory name")
    b = root / BENCH / name
    if not (b.is_dir() and _inside(root, b)):
        raise FileNotFoundError(f"no fleetsim bench {name!r}")
    plan = {r["name"]: (k, r) for k, r in enumerate(_bench_summary(b, root).get("runs") or [])}
    out = []
    for d in _run_dirs(b, root):
        row = _row(d, f"{BENCH}/{name}/{d.name}", root)
        k, rec = plan.get(d.name, (len(plan), {}))
        row.update(variant=rec.get("variant"), status=rec.get("status"), order=k)
        out.append(row)
    out.sort(key=lambda r: (r.pop("order"), r["dir"]))
    return out


def pack(name: str, root: Path | None = None) -> dict:
    root = Path(root or RESULTS_DIR)
    d = run_dir(name, root)
    ev_path = d / "events.jsonl.gz"
    mtime = ev_path.stat().st_mtime
    hit = _cache.get(name)
    if hit and hit[0] == mtime:
        return hit[1]
    out = _pack(d, ev_path, root)
    out["dir"] = name
    _cache[name] = (mtime, out)
    while len(_cache) > _CACHE_N:
        _cache.pop(next(iter(_cache)))
    return out


def _r(v, n=2):
    return None if v is None else round(float(v), n)


def _kind_order(kinds) -> list[str]:
    known = [k for k in KIND_ORDER if k in kinds]
    return known + sorted(k for k in kinds if k not in KIND_ORDER)


def _comms(rows: list[dict], net_cfg: dict) -> dict:
    """The net samples as the monitor reads them. A row per sample:
    [t, dt, load_mean, load_max, over_cap, ogm_air, *per kind KIND_FIELDS];
    counts are over dt (the page divides), null where the run didn't record
    it. A node row per sample: [t, node_load | null, node_air | null, per
    kind | null], per kind (in `kinds` order) null when that kind moved
    nothing, else NODE_FIELDS lists, one value per node, null for all zeros.
    Positional, not the recorded dicts: the key names were a quarter of a
    10-robot run's monitor data (2026-11-26)."""
    kinds = _kind_order({k for ev in rows for k in (ev.get("kinds") or {})})
    # What the run recorded at all: a kind missing from one sample moved nothing
    # then (0), a field the run never recorded (msgs, air_ms before 2026-11-26;
    # air_ms on an ideal network) is unknown (null).
    recorded = {f for ev in rows for st in (ev.get("kinds") or {}).values() for f in st}
    zero = {f: (0 if f in recorded and f != "p95_ms" else None) for f in KIND_FIELDS}
    out_rows, nodes = [], []
    prev = 0.0
    for ev in rows:
        t = float(ev.get("t", 0.0))
        dt = t - prev
        # The end-of-run sample can land a sliver after the last regular one:
        # its few copies over ~0 s would draw as a spike.
        if dt < 0.25:
            continue
        prev = t
        ks = ev.get("kinds") or {}
        row = [round(t, 2), round(dt, 3), ev.get("load_mean"), ev.get("load_max"),
               ev.get("over_cap"), ev.get("ogm_air")]
        for k in kinds:
            st = ks.get(k) or {}
            row += [st.get(f, zero[f]) for f in KIND_FIELDS]
        out_rows.append(row)
        nk = ev.get("node_kinds")
        per = None
        if nk is not None:
            per = [None if k not in nk else [nk[k].get(f) for f in NODE_FIELDS] for k in kinds]
        nodes.append([round(t, 2), ev.get("node_load"), ev.get("node_air"), per])
    radio = net_cfg.get("radio") or {}
    return {"kinds": kinds, "fields": list(KIND_FIELDS), "node_fields": list(NODE_FIELDS),
            "rows": out_rows, "nodes": nodes,
            "model": net_cfg.get("model"), "capacity": net_cfg.get("capacity") or "enforced",
            "contention": radio.get("contention") if net_cfg.get("model") == "radio" else None,
            "cap": LOAD_CAP}


def _pack(d: Path, ev_path: Path, root: Path) -> dict:
    scn = _json(d / "scenario.json", root) or {}
    cfg = scn.get("config") or {}
    n = int((cfg.get("robots") or {}).get("count") or 0)
    rids = [f"robot_{k}" for k in range(n)]
    rix = {r: k for k, r in enumerate(rids)}
    world = None
    actor_ids: list[str] = []
    poses: list[list] = []      # [t, x0, y0, yaw0, x1, ...]
    apose: list[list] = []      # [t, x, y, ...] per actor_ids, null when absent
    paths: dict = {r: [] for r in rids}     # [t, purpose, pts]
    states: dict = {r: [] for r in rids}    # [t, state, task_id]
    marks: list[list] = []      # [t, kind, rid|None, text, x|None, y|None]
    tasks: dict = {}            # task_id -> {pickup, dropoff, t, rid, state}
    board, net, links, belief = [], [], [], []
    net_raw: list[dict] = []    # the net events whole, for _comms
    bmaps: dict = {}
    disturb: list[dict] = []
    t_end = 0.0
    truncated = True            # no `end` event: the run died or is still going
    last_pose: dict = {}        # rid -> (x, y) at the last gt, to place markers

    for ev in ev_lib.read(ev_path):
        s, t = ev["s"], float(ev.get("t", 0.0))
        t_end = max(t_end, t)
        if s == "gt":
            row = [round(t, 2)]
            rob = ev.get("robots") or {}
            for r in rids:
                p = rob.get(r)
                if p is None:
                    row += [None, None, None]
                else:
                    row += [_r(p["x"]), _r(p["y"]), _r(p["yaw"])]
                    last_pose[r] = (p["x"], p["y"])
            poses.append(row)
            act = ev.get("actors") or {}
            for a in act:
                if a not in actor_ids:
                    actor_ids.append(a)
            if actor_ids:
                arow = [round(t, 2)]
                for a in actor_ids:
                    p = act.get(a)
                    arow += [None, None] if p is None else [_r(p.get("x")), _r(p.get("y"))]
                apose.append(arow)
        elif s == "world":
            world = {k: ev[k] for k in ("grid", "waypoints", "homes", "gateway", "body_r", "actors")
                     if k in ev}
        elif s == "path":
            if ev.get("rid") in paths:
                paths[ev["rid"]].append([round(t, 2), ev.get("purpose"), ev.get("pts") or []])
        elif s == "task":
            rid = ev.get("rid")
            if rid in states:
                st = ev.get("state")
                states[rid].append([round(t, 2), st, ev.get("task_id")])
                tid = ev.get("task_id")
                if tid in tasks:
                    tasks[tid]["state"], tasks[tid]["rid"] = st, rid
                if st in TERMINAL:
                    # task_manager goes IDLE straight after; the task event for
                    # that never comes (no task), so it is implied here.
                    states[rid].append([round(t, 2) + 0.001, "IDLE", None])
                    if st in ("DONE", "FAILED"):
                        xy = last_pose.get(rid, (None, None))
                        marks.append([round(t, 2), "done" if st == "DONE" else "failed", rid,
                                      f"{tid} {st.lower()}", _r(xy[0]), _r(xy[1])])
        elif s == "announce":
            tid = ev.get("task_id")
            tasks[tid] = {"pickup": ev.get("pickup"), "dropoff": ev.get("dropoff"),
                          "t": round(t, 2), "rid": None, "state": "announced"}
        elif s == "dispatch":
            tid = ev.get("task_id")
            if tid in tasks:
                tasks[tid]["rid"] = ev.get("rid")
        elif s == "contact":
            other = ev.get("other")
            text = f"{ev.get('kind')}{'' if other is None else ' ' + str(other)}"
            if ev.get("by") == "actor":
                text += " (walked into it)"
            marks.append([round(t, 2), "contact", ev.get("rid"), text, _r(ev.get("x")), _r(ev.get("y"))])
        elif s == "nav":
            e = ev.get("event")
            if e in NAV_MARKS:
                rid = ev.get("rid")
                xy = last_pose.get(rid, (None, None))
                extra = ev.get("cause") or ev.get("by") or ev.get("task_id") or ""
                marks.append([round(t, 2), e, rid, f"{e.replace('_', ' ')}"
                              + (f" ({extra})" if extra else ""), _r(xy[0]), _r(xy[1])])
        elif s == "disturb":
            body = {k: v for k, v in ev.items() if k not in ("s", "t")}
            disturb.append({"t": round(t, 2), **body})
            kind = next((k for k in ("kill", "revive", "partition", "block_link", "blockage")
                         if k in body), "disturb")
            marks.append([round(t, 2), "disturb", None,
                          f"{kind.replace('_', ' ')} {'starts' if body.get('on') else 'ends'}",
                          None, None])
        elif s == "board":
            board.append([round(t, 2), ev.get("tasks") or {}, ev.get("alarms", 0),
                          ev.get("robots_alive")])
        elif s == "net":
            kinds = ev.get("kinds") or {}
            sent = sum(k.get("sent", 0) for k in kinds.values())
            lost = sum(k.get("lost", 0) for k in kinds.values())
            nbytes = sum(k.get("bytes", 0) for k in kinds.values())
            net.append([round(t, 2), ev.get("load_max"), ev.get("reach_frac"),
                        round(lost / sent, 4) if sent else 0.0, nbytes])
            net_raw.append(ev)
        elif s == "links":
            links.append([round(t, 2), [x for e in ev.get("links") or [] for x in e]])
        elif s == "belief":
            rows = ev.get("robots") or {}
            belief.append([round(t, 2), [[_r(rows[r].get("known_free"), 3), rows[r].get("stale")]
                                         if r in rows else None for r in rids]])
        elif s == "belief_map":
            bmaps.setdefault(ev.get("rid"), []).append([round(t, 2), ev.get("grid")])
        elif s == "end":
            truncated = False

    if world is None:
        raise ReplayError(f"{d.name} was recorded before fleetsim recorded its floor "
                          "(2026-11-03); re-run the scenario with the same seed to replay it")
    sc = _json(d / "score.json", root)
    return {
        "dir": d.name,
        "name": scn.get("name") or d.name,
        "title": scn.get("title") or "",
        "description": scn.get("description") or "",
        "seed": scn.get("seed"),
        "network": (cfg.get("network") or {}).get("model"),
        "robots": rids,
        "t_end": round(t_end, 2),
        "truncated": truncated,
        "summary": _json(d / "summary.json", root),
        "score": None if sc is None else {
            "ok": sc.get("ok"),
            "metrics": [{"id": m.get("id"), "ok": m.get("ok"), "values": m.get("values")}
                        for m in sc.get("metrics") or []]},
        "world": world,
        "poses": poses,
        "actor_ids": actor_ids,
        "apose": apose,
        "paths": paths,
        "states": states,
        "tasks": tasks,
        "marks": marks,
        "board": board,
        "net": net,
        "comms": _comms(net_raw, cfg.get("network") or {}),
        "links": links,
        "belief": belief,
        "belief_maps": bmaps,
        "disturb": disturb,
    }
