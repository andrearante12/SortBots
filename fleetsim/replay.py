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

# A run directory NAME (never a path): what `fleetsim run` writes, or --out's
# basename. No `/` and no leading dot, so it can't leave RESULTS_DIR.
DIR_RX = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]{0,127}$")
# Local-layer events worth a marker on the timeline; replans are far too
# frequent to mark one by one (hundreds a minute at 50 robots).
NAV_MARKS = {"blocked", "gave_up", "plan_failed", "empty_shelf", "recovery"}
TERMINAL = {"DONE", "FAILED", "CANCELLED", "REJECTED"}

_cache: dict = {}  # dir name -> (mtime, packed); a handful of runs at most
_CACHE_N = 4


class ReplayError(ValueError):
    pass


def run_dir(name: str, root: Path | None = None) -> Path:
    root = Path(root or RESULTS_DIR)
    if not DIR_RX.match(name or ""):
        raise ReplayError("dir must be a fleetsim run directory name")
    d = root / name
    if not (d / "events.jsonl.gz").is_file():
        raise FileNotFoundError(f"no fleetsim run {name!r}")  # never the path: serve.py is public
    return d


def _json(p: Path):
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


def list_runs(root: Path | None = None, limit: int = 50) -> list[dict]:
    root = Path(root or RESULTS_DIR)
    if not root.is_dir():
        return []
    dirs = sorted((d for d in root.iterdir() if d.is_dir() and DIR_RX.match(d.name)
                   and (d / "events.jsonl.gz").is_file()),
                  key=lambda d: (d / "events.jsonl.gz").stat().st_mtime, reverse=True)
    out = []
    for d in dirs[:limit]:
        scn = _json(d / "scenario.json") or {}
        summ = _json(d / "summary.json")
        sc = _json(d / "score.json")
        cfg = scn.get("config") or {}
        out.append({
            "dir": d.name,
            "name": scn.get("name") or d.name,
            "title": scn.get("title") or "",
            "robots": (cfg.get("robots") or {}).get("count"),
            "network": (cfg.get("network") or {}).get("model"),
            "seed": scn.get("seed"),
            "sim_s": summ.get("sim_s") if summ else None,
            "reason": summ.get("reason") if summ else None,
            # No summary.json: still running, or it died (README "Known issues").
            "finished": summ is not None,
            "verdict": None if sc is None else ("PASS" if sc.get("ok") else "FAIL"),
            "replayable": _has_world(d / "events.jsonl.gz"),
            "mtime": round((d / "events.jsonl.gz").stat().st_mtime, 1),
        })
    return out


def pack(name: str, root: Path | None = None) -> dict:
    d = run_dir(name, root)
    ev_path = d / "events.jsonl.gz"
    mtime = ev_path.stat().st_mtime
    hit = _cache.get(name)
    if hit and hit[0] == mtime:
        return hit[1]
    out = _pack(d, ev_path)
    _cache[name] = (mtime, out)
    while len(_cache) > _CACHE_N:
        _cache.pop(next(iter(_cache)))
    return out


def _r(v, n=2):
    return None if v is None else round(float(v), n)


def _pack(d: Path, ev_path: Path) -> dict:
    scn = _json(d / "scenario.json") or {}
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
    sc = _json(d / "score.json")
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
        "summary": _json(d / "summary.json"),
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
        "links": links,
        "belief": belief,
        "belief_maps": bmaps,
        "disturb": disturb,
    }
