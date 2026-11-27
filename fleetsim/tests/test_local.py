"""fleetsim/local/: the local-avoidance seam (A7), stop_replan and ORCA."""
from __future__ import annotations

import hashlib
import json
import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from fleetsim import config, sim  # noqa: E402
from fleetsim.local import orca as O  # noqa: E402
from fleetsim.local.orca import Orca  # noqa: E402
from fleetsim.local.stop_replan import StopReplan  # noqa: E402
from fleetsim.planner import INF  # noqa: E402

SMALL = {"width_m": 30.0, "height_m": 22.0, "docks": 4, "cross_every_m": 12.0}
OPEN = {**SMALL, "staging_m": 12.0}  # the apron: open floor, x 0.4-29.6, y 0.4-12.4
AISLE_Y = 13.2  # aisle 1 of SMALL, 12.0-14.4 between racks; segment 0 spans x 3.4-13.8


def _sim(homes, local="orca", floor=OPEN, actors=None, duration=60.0, **nav):
    raw = {"name": "t", "seed": 1, "duration_s": duration, "robots": len(homes),
           "config": {"world": {"synthetic": floor, "homes": [list(h) for h in homes]},
                      "map_share": {"prior": "truth"},
                      "nav": {"return_home_s": None, "local": local, **nav}}}
    if actors:
        raw["actors"] = actors
    return sim.Sim(config.resolve(raw))


def _drive(s, goals, at=1.0):
    """Send each robot to its goal, run, and log every step's poses."""
    for r, g in zip(s.robots, goals):
        s.at(at, r.set_goal, g, "home")
    trace = []
    for r in s.robots:
        step = r.step

        def logged(dt, now, r=r, step=step):
            step(dt, now)
            trace.append((now, r.k, r.x, r.y))
        r.step = logged
    moves = []
    try_move = s.try_move

    def logged_move(r, x, y):
        # Whether it left a cell its costmap held free for a lethal one, by
        # the costmap at the time (overlays come and go).
        mult, g = r.cost.mult, s.grid
        into = mult[g.index(x, y)] == INF and mult[g.index(r.x, r.y)] != INF
        ok = try_move(r, x, y)
        if ok:
            moves.append((r.k, x, y, into))
        return ok
    s.try_move = logged_move
    s.run()
    return trace, moves


def _nav(s, event):
    return [e for e in s.events if e["s"] == "nav" and e["event"] == event]


def _min_gap(trace, n):
    """The closest any two robots came, centre to centre."""
    by_t: dict = {}
    for t, k, x, y in trace:
        by_t.setdefault(t, {})[k] = (x, y)
    best = math.inf
    for pos in by_t.values():
        for i in range(n):
            for j in range(i + 1, n):
                if i in pos and j in pos:
                    best = min(best, math.dist(pos[i], pos[j]))
    return best


def _arrived(s, goals):
    """Every robot reached its goal: its path's end, the goal's cell centre
    (up to 0.14 m off), within 5 cm."""
    return all(r.goal is None and math.dist((r.x, r.y), g) <= 0.2 for r, g in zip(s.robots, goals))


# ---- the seam ------------------------------------------------------------------------

def test_the_seam_selects_the_method():
    assert all(isinstance(r.local, StopReplan) for r in _sim([(5, 5), (8, 5)], local="stop_replan").robots)
    assert all(isinstance(r.local, Orca) for r in _sim([(5, 5), (8, 5)], local="orca").robots)
    default = sim.Sim(config.resolve({"name": "t", "seed": 1, "duration_s": 10, "robots": 1,
                                      "config": {"world": {"synthetic": OPEN}}}))
    assert default.cfg["nav"]["local"] == "stop_replan"
    assert isinstance(default.robots[0].local, StopReplan)
    with pytest.raises(config.ScenarioError, match="nav.local"):
        config.resolve({"name": "t", "seed": 1, "duration_s": 10, "robots": 1,
                        "config": {"world": {"synthetic": OPEN}, "nav": {"local": "rvo"}}})


# A run with stops, replans around, bumps, notices and a mover, every default,
# on the tree before the seam (b85effd, 2026-11-26): every event but wall,
# wall_s and summary.
GOLDEN = {"numpy": "2.3.5", "events": 2560,
          "sha256": "5fa6dd9224794f8131082a7a026a89b13bebfdcb2e1c922b5a853e4851faa952"}


def _golden_run():
    tasks = [{"at_s": 1 + 2 * i, "pickup": f"shelf_{1 + i % 3}_{i % 3}", "dropoff": f"dock_{i % 4}"}
             for i in range(10)]
    raw = {"name": "t", "seed": 2, "duration_s": 400, "robots": 6,
           "config": {"world": {"synthetic": SMALL}, "map_share": {"prior": "truth"}},
           "tasks": tasks, "stop": {"until": "tasks_done", "settle_s": 1},
           "actors": [{"id": "w", "shape": "cylinder", "radius": 0.25, "height": 1.75,
                       "path": {"waypoints": [[4.0, AISLE_Y], [13.0, AISLE_Y]], "speed_mps": 1.2,
                                "mode": "pingpong", "dwell_s": 2}}]}
    s = sim.Sim(config.resolve(raw))
    s.run()
    h, n = hashlib.sha256(), 0
    for e in s.events:
        h.update(json.dumps({k: v for k, v in e.items() if k not in ("wall", "wall_s", "summary")},
                            sort_keys=True).encode())
        n += 1
    return s, n, h.hexdigest()


@pytest.mark.skipif(np.__version__ != GOLDEN["numpy"], reason="golden recorded with another numpy")
def test_stop_replan_reproduces_the_event_stream_from_before_the_seam():
    s, n, digest = _golden_run()
    assert _nav(s, "blocked") and _nav(s, "replan")  # the local layer had work to do
    assert (n, digest) == (GOLDEN["events"], GOLDEN["sha256"])


# ---- ORCA: the linear programs -----------------------------------------------------------

def test_orca_solver_keeps_the_preferred_velocity_when_it_is_allowed():
    lines = [O.half_plane(0.0, 1.0, 0.2)]  # v.y <= 0.2
    assert O.solve(lines, 1, 0.5, (0.5, 0.0)) == pytest.approx((0.5, 0.0))
    v = O.solve(lines, 1, 0.5, (0.0, 0.5))
    assert v[1] == pytest.approx(0.2) and abs(v[0]) < 1e-9


def test_orca_fallback_never_gives_up_a_wall_line():
    # Two agent lines that leave nothing (v.x >= 0.4 and v.x <= -0.4) and a
    # hard wall line v.y <= 0: the 3D program trades the agents off, never
    # the wall.
    agents = [O.half_plane(-1.0, 0.0, -0.4), O.half_plane(1.0, 0.0, -0.4)]
    wall = [O.half_plane(0.0, 1.0, 0.0)]
    v = O.solve(wall + agents, 1, 0.5, (0.0, 0.5))
    assert v[1] <= 1e-6


def test_orca_head_on_lines_send_both_to_their_own_right():
    # Two robots 3 m apart closing at 1 m/s on one line: each line, alone,
    # moves the velocity off the line to the robot's own right.
    a = O.agent_line(3.0, 0.0, 1.0, 0.0, 0.6, 1 / 4.0, 0.5, 0.5, 0.0)
    b = O.agent_line(-3.0, 0.0, -1.0, 0.0, 0.6, 1 / 4.0, 0.5, -0.5, 0.0)
    va = O.solve([a], 0, 0.5, (0.5, 0.0))
    vb = O.solve([b], 0, 0.5, (-0.5, 0.0))
    assert va[1] < -0.01 and vb[1] > 0.01  # east-bound goes south, west-bound north


# ---- ORCA in the sim ----------------------------------------------------------------------

def test_orca_two_robots_head_on_in_open_space_pass_without_contact_or_stopping():
    homes, goals = [(6.0, 6.0), (20.0, 6.0)], [(20.0, 6.0), (6.0, 6.0)]
    s = _sim(homes)
    trace, _ = _drive(s, goals)
    assert _arrived(s, goals)
    assert s.counts["contacts"] == 0
    assert not _nav(s, "blocked") and not _nav(s, "gave_up")
    assert _min_gap(trace, 2) >= 0.5 + 0.05  # kept clear, not just out of contact
    # The same meeting under stop_replan stops (it swerves only by replanning).
    s2 = _sim(homes, local="stop_replan")
    _drive(s2, goals)
    assert _nav(s2, "blocked")


def test_orca_four_robots_crossing_at_a_point_all_get_through():
    c = (15.0, 6.4)
    homes = [(c[0] - 4, c[1]), (c[0] + 4, c[1]), (c[0], c[1] - 4), (c[0], c[1] + 4)]
    goals = [homes[1], homes[0], homes[3], homes[2]]
    s = _sim(homes)
    trace, _ = _drive(s, goals)
    assert _arrived(s, goals)
    assert s.counts["contacts"] == 0
    assert not _nav(s, "gave_up")
    assert _min_gap(trace, 4) >= 0.5


# The robot drives east along y = 6 and passes x = 17 at about t = 23.4 s.
# The walker crosses its line there at 1.2 m/s, diagonally from ahead and to
# the right, so the camera has it in view on its way in: a walker crossing
# square on at that speed comes in at 67 degrees off the heading, outside
# the 87 degree cone, and no local layer could see it in time.
CROSS, U = (17.0, 6.0), (-math.sqrt(0.5), math.sqrt(0.5))
W_FROM = (CROSS[0] - 5 * U[0], CROSS[1] - 5 * U[1])
W_TO = (CROSS[0] + 4 * U[0], CROSS[1] + 4 * U[1])
W_START = 23.4 - 5 / 1.2


def _walker_at(t):
    f = min(max((t - W_START) * 1.2, 0.0), 9.0)
    return (W_FROM[0] + f * U[0], W_FROM[1] + f * U[1])


def _walker_run(with_walker: bool, local="orca"):
    actors = [{"id": "w", "shape": "cylinder", "radius": 0.25, "height": 1.75,
               "path": {"waypoints": [list(W_FROM), list(W_TO)], "speed_mps": 1.2,
                        "mode": "once", "start_at_s": W_START}}] if with_walker else None
    s = _sim([(6.0, 6.0)], local=local, actors=actors, duration=50.0)
    trace, _ = _drive(s, [(24.0, 6.0)])
    done = next(e["t"] for e in s.events if e["s"] == "path" and not e["pts"])
    return s, trace, done


def test_orca_avoids_a_walker_crossing_in_front():
    _free, free_trace, free_done = _walker_run(False)
    # Without avoidance the two would meet: the walker passes within a body
    # of where the undisturbed robot is.
    assert min(math.dist((x, y), _walker_at(t)) for t, _k, x, y in free_trace) < 0.5
    s, trace, done = _walker_run(True)
    assert s.counts["contacts"] == 0
    assert _arrived(s, [(24.0, 6.0)])
    assert min(math.dist((x, y), _walker_at(t)) for t, _k, x, y in trace) >= 0.25 + 0.25
    assert done > free_done  # it gave way: slowed or went round


def test_orca_never_steers_into_a_wall():
    # Head-on on one line 0.5 m off the aisle's centre: one of the two
    # dodges toward the rack face.
    y = AISLE_Y + 0.5
    homes, goals = [(4.6, y), (12.6, y)], [(12.6, y), (4.6, y)]
    s = _sim(homes, floor=SMALL)
    trace, moves = _drive(s, goals)
    assert s.counts["contacts"] == 0
    assert _arrived(s, goals)
    assert max(yy for _t, _k, _x, yy in trace) > y + 0.2  # it did go toward the rack
    assert not any(into for *_m, into in moves)
    # And a busy run: six robots on deliveries, a walker pacing an aisle, a
    # single-lane aisle. No robot ever drives into a wall or a rack, or from
    # free space into a lethal cell of its own costmap.
    tasks = [{"at_s": 1 + 2 * i, "pickup": f"shelf_{1 + i % 3}_{i % 3}", "dropoff": f"dock_{i % 4}"}
             for i in range(10)]
    raw = {"name": "t", "seed": 2, "duration_s": 400, "robots": 6,
           "config": {"world": {"synthetic": {**SMALL, "narrow_aisles": [2]}},
                      "map_share": {"prior": "truth"}, "nav": {"local": "orca"}},
           "tasks": tasks, "stop": {"until": "tasks_done", "settle_s": 1},
           "actors": [{"id": "w", "shape": "cylinder", "radius": 0.25, "height": 1.75,
                       "path": {"waypoints": [[4.0, AISLE_Y], [13.0, AISLE_Y]], "speed_mps": 1.2,
                                "mode": "pingpong", "dwell_s": 2}}]}
    s = sim.Sim(config.resolve(raw))
    _trace, moves = _drive(s, [])
    assert sum(e["s"] == "task" and e["state"] == "DONE" for e in s.events) == 10
    assert not [e for e in s.events if e["s"] == "contact" and e["kind"] == "static"]
    assert not any(into for *_m, into in moves)


def test_orca_wall_lines_hold_back_only_a_dodge_toward_the_wall():
    # 0.5 m from the centres of the rack face's cells, north of the robot.
    s = _sim([(4.6, 14.0)], floor=SMALL)
    r = s.robots[0]
    s.at(0.0, r.set_goal, (12.6, AISLE_Y), "home")
    s.duration = 0.2
    s.run()
    r.x, r.y = 4.7, 14.0
    pref = r.preferred_velocity()
    lines = r.local.wall_lines(pref)
    assert lines
    # The preferred velocity (along the aisle) is allowed by every wall line...
    assert all(ln[2] * (ln[1] - pref[1]) - ln[3] * (ln[0] - pref[0]) <= 1e-9 for ln in lines)
    # ...and a dodge straight at the rack face is held to (0.5 m - lethal
    # radius) / wall_horizon_s.
    v = O.solve(lines, len(lines), r.vmax, (0.0, 0.5))
    bound = (0.5 - r.cost.r_lethal) / float(r.nav["orca"]["wall_horizon_s"])
    assert v[1] == pytest.approx(bound, abs=1e-6)


def test_orca_runs_are_reproducible():
    homes = [(6.0, 6.0), (20.0, 6.0), (13.0, 2.0)]
    goals = [(20.0, 6.0), (6.0, 6.0), (13.0, 10.0)]
    a, b = _sim(homes), _sim(homes)
    _drive(a, goals)
    _drive(b, goals)
    strip = lambda evs: [{k: v for k, v in e.items() if k not in ("wall", "wall_s", "summary")}  # noqa: E731
                         for e in evs]
    assert strip(a.events) == strip(b.events)
