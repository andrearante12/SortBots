"""A scenario's floor without a run: what the scenario editor draws and checks.

    /usr/bin/python3 -m fleetsim.preview NAME     # a saved scenario: its floor and its problems
    /usr/bin/python3 -m fleetsim.preview -        # JSON on stdin, JSON on stdout (serve.py)

The floor, waypoints, gateway and parking spots come from `Sim.world_only`,
which calls the sim's own `_build_world` and `_homes`: whatever a run of the
scenario would get, synthetic options and saved maps alike, with nothing
copied that could drift. Building them takes milliseconds; nothing is
simulated.

`problems()` are the mistakes only the floor can show, which `config.resolve`
can't see because it never builds one: a task naming a waypoint that isn't
there (fleet_tasks rejects it silently), a `task_arrival` pattern matching
nothing, a blockage off the map, a disturbance naming a robot the fleet
doesn't have. Each one makes a run crash or quietly do less than it says.
Level `error` ones block the editor's Save; `warning` ones are shown only.

The operator view's editor (webui/operator_fleetedit.js) gets this through
webui/fleetsim_editor.py, which runs it as a /usr/bin/python3 subprocess:
serve.py itself needs no numpy (a console started from conda may have none),
and a child can be held to a time and memory limit, which matters because the
scenario comes from a page that may be on the internet. That module refuses
paths and maps outside the library before anything reaches this one.

Stdin is {"scenario": <raw mapping>, "grid": bool}; stdout is
{"world": {...}, "problems": [...], "summary": {...}} or {"error", "where"}.
"""
from __future__ import annotations

import fnmatch
import json
import re
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from fleetsim import config  # noqa: E402
from fleetsim import grid as G  # noqa: E402
from fleetsim import sim as sim_lib  # noqa: E402
from fleetsim.planner import FREE, OCC  # noqa: E402
from simtest import world as world_lib  # noqa: E402
from simtest.events import MapGrid  # noqa: E402

ROBOT_RX = re.compile(r"^robot_(\d+)$")


def build(scn: dict):
    """(world-only Sim, homes, error building the homes or None)."""
    s = sim_lib.Sim.world_only(scn)
    n = int(scn["config"]["robots"]["count"])
    try:
        return s, s._homes(n), None
    except ValueError as e:  # "room for only N robots on this map"
        return s, list(s._map_homes)[:n], str(e)


def world_json(scn: dict, s, homes, grid: bool = True) -> dict:
    g, w = s.grid, scn["config"]["world"]
    out = {
        "source": w["source"], "map": w["map"], "cell_m": g.res,
        "origin": [g.ox, g.oy], "size_m": [round(g.width * g.res, 3), round(g.height * g.res, 3)],
        "waypoints": {k: [round(v.x, 3), round(v.y, 3)] for k, v in s.waypoints.items()},
        "homes": [[round(float(x), 3), round(float(y), 3)] for x, y in homes],
        "gateway": [round(float(v), 3) for v in s.gateway_xy],
        "body_r": float(scn["config"]["robots"]["footprint_radius_m"]),
        "free_lost": round(float(s.free_lost), 4),
        "notes": list(s.notes),
        # So the page can offer every synthetic option, including ones added
        # after it was written, with its default as the placeholder.
        "synthetic_defaults": dict(G.SYNTHETIC_DEFAULTS),
        "actors": [{"id": a["id"], "r": round(float(world_lib.half_extent(a)), 3),
                    "path": {k: (v if k != "waypoints" else [list(p) for p in v])
                             for k, v in a["path"].items()}}
                   for a in s.actors],
    }
    if grid:
        out["grid"] = MapGrid(g.width, g.height, g.res, g.ox, g.oy,
                              np.where(s.static, OCC, FREE).astype(np.int8)).to_json()
    return out


def _num(v):
    if isinstance(v, bool) or not isinstance(v, (int, float)):
        return None
    return float(v)


def problems(scn: dict, s, homes_err: str | None) -> list[dict]:
    out: list[dict] = []

    def bad(where, msg, level="error"):
        out.append({"where": where, "error": msg, "level": level})

    cfg = scn["config"]
    n = int(cfg["robots"]["count"])
    dur = float(scn["duration_s"])
    wps = s.waypoints
    g = s.grid
    if homes_err:
        bad("robots", homes_err)

    def at_ok(where, v):
        t = _num(v)
        if t is None:
            bad(where, "expected a number of seconds")
        elif t < 0:
            bad(where, "can't be before the run starts")
        elif t >= dur:
            bad(where, f"at or after the end of the run ({dur:g} s), so it never happens", "warning")

    for i, t in enumerate(scn["tasks"]):
        at_ok(f"tasks[{i}].at_s", t.get("at_s"))
        for k in ("pickup", "dropoff"):
            if t.get(k) not in wps:
                bad(f"tasks[{i}].{k}", f"no waypoint {t.get(k)!r} on this floor "
                                       "(fleet_tasks would reject the task)")
    arr = scn["task_arrival"]
    if arr:
        names = list(wps)
        for k in ("pickups", "dropoffs"):
            pat = arr.get(k, "*")
            if not isinstance(pat, str):
                bad(f"task_arrival.{k}", "expected a waypoint name pattern (fnmatch)")
            elif not any(fnmatch.fnmatch(x, pat) for x in names):
                bad(f"task_arrival.{k}", f"{pat!r} matches no waypoint on this floor")
        for k in ("start_s", "until_s"):
            if k in arr and arr[k] is not None and _num(arr[k]) is None:
                bad(f"task_arrival.{k}", "expected a number of seconds")

    # The gateway is a node a partition or a cut link can name. `kill: gateway`
    # is accepted too: a dead gateway is being added (docs/fleetsim-plan.md C12),
    # and a check stricter than the sim would refuse those scenarios here.
    def node_ok(where, name):
        if name == "gateway":
            return
        m = ROBOT_RX.match(name) if isinstance(name, str) else None
        if not m or int(m.group(1)) >= n:
            bad(where, f"{name!r} is not a node of this fleet (robot_0..robot_{n - 1}, gateway)")

    for i, d in enumerate(scn["disturbances"]):
        w = f"disturbances[{i}]"
        at_ok(f"{w}.at_s", d.get("at_s"))
        if d.get("duration_s") is not None:
            v = _num(d["duration_s"])
            if v is None or v <= 0:
                bad(f"{w}.duration_s", "expected seconds > 0 (or leave it out: no end)")
        if "kill" in d:
            node_ok(f"{w}.kill", d["kill"])
        elif "revive" in d:
            node_ok(f"{w}.revive", d["revive"])
        elif "block_link" in d:
            pair = d["block_link"]
            if not isinstance(pair, list) or len(pair) != 2 or pair[0] == pair[1]:
                bad(f"{w}.block_link", "expected two different nodes, [a, b]")
            else:
                for j, x in enumerate(pair):
                    node_ok(f"{w}.block_link[{j}]", x)
        elif "partition" in d:
            groups = d["partition"]
            if not isinstance(groups, list) or len(groups) < 2:
                bad(f"{w}.partition", "expected two or more groups, e.g. [[robot_0, robot_1], rest]")
            else:
                seen: set = set()
                for j, grp in enumerate(groups):
                    if grp == "rest":
                        continue
                    if not isinstance(grp, list) or not grp:
                        bad(f"{w}.partition[{j}]", "a group is a list of nodes, or rest")
                        continue
                    for x in grp:
                        node_ok(f"{w}.partition[{j}]", x)
                        if x in seen:
                            bad(f"{w}.partition[{j}]", f"{x!r} is in two groups")
                        seen.add(x)
        elif "blockage" in d:
            b = d["blockage"]
            rect = b.get("rect") if isinstance(b, dict) else None
            nums = [_num(v) for v in rect] if isinstance(rect, list) else []
            if len(nums) != 4 or None in nums:
                bad(f"{w}.blockage", "expected {rect: [x0, y0, x1, y1]} in metres")
            else:
                x0, y0, x1, y1 = nums
                if g.cell(min(x0, x1), min(y0, y1)) is None or g.cell(max(x0, x1), max(y0, y1)) is None:
                    bad(f"{w}.blockage", f"off the map: the floor is {g.ox:g}..{g.ox + g.width * g.res:g} x "
                                         f"{g.oy:g}..{g.oy + g.height * g.res:g} m")
    x_hi, y_hi = g.ox + g.width * g.res, g.oy + g.height * g.res
    for i, a in enumerate(scn.get("actors") or []):
        for j, (x, y) in enumerate(a["path"]["waypoints"]):
            if not (g.ox <= x <= x_hi and g.oy <= y <= y_hi):
                bad(f"actors[{i}].path", f"point {j} ({x:g}, {y:g}) is off the floor", "warning")
                break
    want = cfg["world"].get("gateway")
    if want and [round(float(v), 3) for v in want] != [round(float(v), 3) for v in s.gateway_xy]:
        bad("config.world.gateway", "not used on this floor: the gateway is at "
                                    f"({s.gateway_xy[0]:g}, {s.gateway_xy[1]:g})", "warning")
    return out


def summary(scn: dict, s, homes) -> dict:
    arr = scn["task_arrival"] or {}
    expected = None
    if arr.get("rate_hz"):
        start = float(arr.get("start_s", 0.0) or 0.0)
        until = float(arr.get("until_s", scn["duration_s"]) or scn["duration_s"])
        expected = max(0.0, (min(until, scn["duration_s"]) - start)) * float(arr["rate_hz"])
        if arr.get("count") is not None:
            expected = min(expected, float(arr["count"]))
        expected = round(expected, 1)
    g = s.grid
    return {"robots": int(scn["config"]["robots"]["count"]), "duration_s": scn["duration_s"],
            "network": scn["config"]["network"]["model"], "tasks": len(scn["tasks"]),
            "arrival_expected": expected, "disturbances": len(scn["disturbances"]),
            "actors": len(scn.get("actors") or []), "metrics": len(scn["metrics"]),
            "waypoints": len(s.waypoints), "homes": len(homes),
            "floor_m": [round(g.width * g.res, 2), round(g.height * g.res, 2)], "cell_m": g.res}


def check(raw: dict, grid: bool = True) -> dict:
    """A raw scenario mapping -> what the editor shows. Never raises for a
    bad scenario: the error comes back in the result."""
    try:
        scn = config.resolve(raw)
    except config.ScenarioError as e:
        return {"error": str(e)}
    try:
        s, homes, homes_err = build(scn)
    except (ValueError, OSError, KeyError) as e:
        where = "config.world.map" if scn["config"]["world"]["source"] == "map" else "config.world"
        return {"error": f"can't build the floor: {e}", "where": where}
    return {"world": world_json(scn, s, homes, grid=grid),
            "problems": problems(scn, s, homes_err),
            "summary": summary(scn, s, homes)}


# The serve.py child's limits. A floor's size is checked before it gets here
# (webui/fleetsim_editor.py MAX_CELLS); these catch whatever that misses.
CHILD_MEM_B = 2 << 30
CHILD_CPU_S = 30


def main(argv: list[str]) -> int:
    if argv == ["-"]:
        import resource

        resource.setrlimit(resource.RLIMIT_AS, (CHILD_MEM_B, CHILD_MEM_B))
        resource.setrlimit(resource.RLIMIT_CPU, (CHILD_CPU_S, CHILD_CPU_S))
        req = json.load(sys.stdin)
        out = check(req.get("scenario"), grid=bool(req.get("grid", True)))
        json.dump(out, sys.stdout, separators=(",", ":"))
        return 0
    if len(argv) != 1:
        print("\n".join(__doc__.strip().splitlines()[2:4]), file=sys.stderr)
        return 2
    try:
        raw = config.load(argv[0])
    except config.ScenarioError as e:
        print(f"invalid: {e}", file=sys.stderr)
        return 2
    # load() resolved it; check() wants the raw mapping, so re-read the file.
    import yaml

    out = check(yaml.safe_load(config.path_for(argv[0]).read_text()), grid=False)
    if "error" in out:
        print(f"invalid: {out['error']}", file=sys.stderr)
        return 2
    w, sm = out["world"], out["summary"]
    print(f"{argv[0]}: {w['source']} floor {sm['floor_m'][0]:g} x {sm['floor_m'][1]:g} m at "
          f"{sm['cell_m']:g} m cells, {sm['waypoints']} waypoints, {sm['homes']} homes, "
          f"gateway at ({w['gateway'][0]:g}, {w['gateway'][1]:g})")
    for note in w["notes"]:
        print(f"  note: {note}")
    for p in out["problems"]:
        print(f"  {p['level']}: {p['where']}: {p['error']}")
    return 1 if any(p["level"] == "error" for p in out["problems"]) else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
