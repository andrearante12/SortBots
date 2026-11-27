"""Robot types and capabilities (A6): per-type speed and service times, tasks
that require a capability, under the auction and both central dispatchers.
Small floor, seconds per test."""
from __future__ import annotations

import collections
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from fleetsim import config, sim  # noqa: E402
from test_sim import run, scenario, score  # noqa: E402

METHODS = ("auction", "central_greedy", "central_hungarian")
HEAVY = [{"name": "standard"}, {"name": "heavy", "count": 1, "capabilities": ["heavy"]}]


def typed(types, method="auction", **over):
    cfg = sim.deep_merge({"robots": {"types": types}, "dispatch": {"method": method}},
                         over.pop("config", {}))
    return scenario(config=cfg, **over)


def _types(s) -> dict:
    return next(e for e in s.events if e["s"] == "world")["types"]


# ---- the fleet a type list makes --------------------------------------------------

def _counts(types, n):
    cfg = config.load_base()
    cfg["robots"]["count"], cfg["robots"]["types"] = n, types
    return config.robot_profiles(cfg)


MIXED = [{"name": "standard"}, {"name": "fast", "share": 0.3, "max_speed_mps": 0.8},
         {"name": "heavy", "share": 0.2, "capabilities": ["heavy"], "pick_s": 6.0}]


@pytest.mark.parametrize("n, want", [(2, {"fast": 1, "heavy": 1}),
                                     (5, {"standard": 2, "fast": 2, "heavy": 1}),
                                     (10, {"standard": 5, "fast": 3, "heavy": 2}),
                                     (20, {"standard": 10, "fast": 6, "heavy": 4})])
def test_type_shares_scale_with_the_fleet(n, want):
    prof = _counts(MIXED, n)
    assert len(prof) == n
    got = collections.Counter(p["type"] for p in prof)
    assert {k: v for k, v in got.items() if v} == want
    fast = next(p for p in prof if p["type"] == "fast")
    assert fast["max_speed_mps"] == 0.8 and fast["pick_s"] == 2.0 and fast["capabilities"] == ()
    assert next(p for p in prof if p["type"] == "heavy")["capabilities"] == ("heavy",)


def test_types_are_spread_through_the_fleet_not_in_blocks():
    # robot_k parks at the k-th home and wins exact ties by index: a block
    # would put all of a type at one end of the apron.
    order = [p["type"] for p in _counts([{"name": "a"}, {"name": "b", "share": 0.5}], 10)]
    assert order.count("a") == order.count("b") == 5
    assert all(order[i] != order[i + 1] for i in range(9))


def test_a_fixed_count_does_not_scale_and_one_type_takes_the_rest():
    for n in (3, 8):
        got = collections.Counter(p["type"] for p in _counts(HEAVY, n))
        assert got == {"standard": n - 1, "heavy": 1}


def test_untyped_robots_are_the_base_values():
    cfg = config.load_base()
    cfg["robots"]["count"] = 3
    prof = config.robot_profiles(cfg)
    r, d = cfg["robots"], float(cfg["tasks"]["dwell_s"])
    assert prof == [{"type": "default", "max_speed_mps": r["max_speed_mps"],
                     "accel_mps2": r["accel_mps2"], "yaw_rate_rps": r["yaw_rate_rps"],
                     "pick_s": d, "place_s": d, "battery_drain_pct_per_s": None,
                     "capabilities": ()}] * 3


@pytest.mark.parametrize("types, tasks, match", [
    ([{"name": "a"}, {"name": "b"}], [], "at most one type"),
    ([{"name": "a", "share": 0.5, "count": 2}], [], "share or count"),
    ([{"name": "a", "speed": 1.0}], [], "unknown key"),
    ([{"name": "a"}, {"name": "a", "share": 0.2}], [], "unique name"),
    ([{"name": "a", "count": 2}], [], "make 2 robots at a fleet of 3"),
    ([{"name": "a"}, {"name": "b", "count": 4}], [], "take 4 robots, the fleet has 3"),
    ([{"name": "a", "max_speed_mps": 0}], [], "max_speed_mps"),
    (HEAVY, [{"at_s": 1, "pickup": "shelf_1_0", "dropoff": "dock_0", "requires": "cold"}],
     "no robot type has"),
    (None, [{"at_s": 1, "pickup": "shelf_1_0", "dropoff": "dock_0", "requires": "heavy"}],
     "robots.types is not set"),
])
def test_bad_types_and_requirements_fail_validation(types, tasks, match):
    with pytest.raises(config.ScenarioError, match=match):
        typed(types, tasks=tasks or scenario()["tasks"])


def test_arrival_requirements_are_validated():
    arr = {"rate_hz": 0.1, "pickups": "shelf_*", "dropoffs": "dock_*"}
    with pytest.raises(config.ScenarioError, match="more than 1"):
        typed(HEAVY, task_arrival={**arr, "requires": {"heavy": 0.7, "heavier": 0.4}})
    with pytest.raises(config.ScenarioError, match="no robot type has"):
        typed(HEAVY, task_arrival={**arr, "requires": {"cold": 0.2}})


# ---- capabilities, end to end ---------------------------------------------------------

TASKS = [{"at_s": 1, "pickup": "shelf_1_0", "dropoff": "dock_0"},
         {"at_s": 2, "pickup": "shelf_2_2", "dropoff": "dock_2", "requires": "heavy"},
         {"at_s": 3, "pickup": "shelf_3_1", "dropoff": "dock_3"},
         {"at_s": 4, "pickup": "shelf_0_1", "dropoff": "dock_1", "requires": ["heavy"]}]


@pytest.mark.parametrize("method", METHODS)
def test_a_task_that_needs_a_capability_goes_only_to_robots_that_have_it(method):
    s, summary = run(typed(HEAVY, method, robots=4, tasks=TASKS, duration_s=900))
    assert summary["reason"] == "tasks_done"
    types = _types(s)
    assert collections.Counter(types.values()) == {"standard": 3, "heavy": 1}
    need = {e["task_id"] for e in s.events if e["s"] == "announce" and e.get("requires")}
    assert need == {"fleet-2", "fleet-4"}
    disp = [(e["task_id"], e["rid"]) for e in s.events if e["s"] == "dispatch"]
    assert {t for t, _ in disp} == {"fleet-1", "fleet-2", "fleet-3", "fleet-4"}
    assert all(types[r] == "heavy" for t, r in disp if t in need), disp
    v = score(s, [{"metric": "fleet_tasks"}, {"metric": "nav"}, {"metric": "bounds"}])
    assert v["fleet_tasks"]["done"] == 4 and v["nav"]["empty_shelf"] == 0
    assert v["bounds"]["unservable"] == 0


@pytest.mark.parametrize("method", METHODS)
def test_a_task_no_robot_can_do_is_reported_and_the_run_still_ends(method):
    # The heavy type has no robots at this size: its task is announced (as
    # the gateway would), never assigned, noted, and not waited for.
    types = [{"name": "standard"}, {"name": "heavy", "count": 0, "capabilities": ["heavy"]}]
    tasks = TASKS[:3]
    s, summary = run(typed(types, method, tasks=tasks, duration_s=900))
    assert summary["reason"] == "tasks_done" and summary["sim_s"] < 600
    assert any("fleet-2 requires ['heavy']: no robot" in n for n in summary["notes"])
    tb = next(e for e in s.events if e["s"] == "task_bound" and e["task_id"] == "fleet-2")
    assert tb["lb_s"] is None and tb["able"] == 0 and tb["requires"] == ["heavy"]
    assert "fleet-2" not in {e["task_id"] for e in s.events if e["s"] == "dispatch"}
    v = score(s, [{"metric": "fleet_tasks"}, {"metric": "bounds"}])
    assert v["fleet_tasks"]["done"] == 2 and v["fleet_tasks"]["backlog_end"] == 1
    assert v["bounds"]["unservable"] == 1


# ---- per-type speed and service times --------------------------------------------------

SPLIT = [{"name": "a", "capabilities": ["a"], "pick_s": 1.0, "place_s": 1.5},
         {"name": "b", "count": 1, "capabilities": ["b"], "pick_s": 6.0, "place_s": 4.0,
          "max_speed_mps": 0.8, "accel_mps2": 0.8}]


@pytest.mark.parametrize("method", ("auction", "central_hungarian"))
def test_each_robot_picks_places_and_drives_at_its_own_types_values(method):
    tasks = [{"at_s": 1, "pickup": "shelf_1_0", "dropoff": "dock_0", "requires": "a"},
             {"at_s": 1, "pickup": "shelf_3_2", "dropoff": "dock_3", "requires": "b"}]
    s, summary = run(typed(SPLIT, method, robots=2, tasks=tasks, duration_s=600))
    assert summary["reason"] == "tasks_done"
    types = _types(s)
    prof = {t["name"]: t for t in SPLIT}
    last, dwell = {}, collections.defaultdict(list)
    for e in s.events:
        if e["s"] != "task":
            continue
        prev = last.get(e["rid"])
        if prev and prev[0] in ("PICKING", "PLACING") and e["state"] != prev[0]:
            dwell[(types[e["rid"]], prev[0])].append(e["t"] - prev[1])
        if prev is None or prev[0] != e["state"]:
            last[e["rid"]] = (e["state"], e["t"])
    for name, p in prof.items():
        (pick,), (place,) = dwell[(name, "PICKING")], dwell[(name, "PLACING")]
        assert p["pick_s"] <= pick <= p["pick_s"] + 0.11, (name, pick)
        assert p["place_s"] <= place <= p["place_s"] + 0.11, (name, place)
    # Top speed from ground truth (5 Hz): b drives at 0.8, a at the base 0.5
    # (gt poses are rounded to 1 mm, ~1 % of a 0.2 s sample).
    gts = [e for e in s.events if e["s"] == "gt"]
    top = collections.defaultdict(float)
    for g0, g1 in zip(gts, gts[1:]):
        if g1["t"] <= g0["t"]:
            continue  # the run's closing sample repeats the last step
        for rid, p1 in g1["robots"].items():
            p0 = g0["robots"][rid]
            v = ((p1["x"] - p0["x"]) ** 2 + (p1["y"] - p0["y"]) ** 2) ** 0.5 / (g1["t"] - g0["t"])
            top[types[rid]] = max(top[types[rid]], v)
    assert 0.7 < top["b"] <= 0.8 * 1.02 and 0.4 < top["a"] <= 0.5 * 1.02, dict(top)


def test_the_bound_is_the_best_type_that_can_do_the_task():
    types = [{"name": "standard"},
             {"name": "fast", "count": 1, "max_speed_mps": 1.0, "pick_s": 3.0, "place_s": 3.0},
             {"name": "heavy", "count": 1, "capabilities": ["heavy"], "max_speed_mps": 0.25,
              "pick_s": 5.0}]
    s, _ = run(typed(types, robots=3, tasks=TASKS, duration_s=1.0))
    for e in (e for e in s.events if e["s"] == "task_bound"):
        d = e["drive_m"]
        if e.get("requires"):
            want = d / 0.25 + 5.0 + 2.0
            assert e["able"] == 1
        else:
            want = min(d / 0.5 + 4.0, d / 1.0 + 6.0, d / 0.25 + 7.0)
            assert "able" not in e
        assert e["lb_s"] == pytest.approx(want, abs=0.02), e


def test_arrival_requirements_leave_the_task_stream_itself_unchanged():
    # Their own RNG: the same seed gives the same times, pickups and dropoffs
    # with and without `requires`, so a mixed fleet meets the same tasks.
    arr = {"rate_hz": 1.0, "pickups": "shelf_*", "dropoffs": "dock_*", "count": 400}

    def stream(scn):
        s = sim.Sim(scn)
        return [(t, a[0], a[1], a[3] if len(a) > 3 else ()) for t, _q, fn, a in sorted(s._heap)
                if getattr(fn, "__name__", "") == "_announce"]

    plain = stream(typed(HEAVY, tasks=[], task_arrival=arr))
    mixed = stream(typed(HEAVY, tasks=[], task_arrival={**arr, "requires": {"heavy": 0.25}}))
    assert [x[:3] for x in plain] == [x[:3] for x in mixed] and len(plain) == 400
    assert not any(x[3] for x in plain)
    share = sum(x[3] == ("heavy",) for x in mixed) / len(mixed)
    assert 0.18 < share < 0.32


# ---- central dispatchers: eligibility and each robot's own costs --------------------------

def _gateway(method, types):
    s = sim.Sim(typed(types, method, robots=2, tasks=[]))
    return s, s.gw


def _near_and_far(s, gw, goal):
    """Two waypoints, the far one 1.3-1.8x the near one's drive from goal."""
    c = gw.costs
    d = {w: float(c.seconds(goal, [c.snap(s.waypoint_xy(w))])[0]) for w in sorted(s.waypoints)}
    for near in sorted(d, key=d.get):
        if d[near] < 20.0:
            continue
        for far in sorted(d, key=d.get):
            if 1.3 * d[near] < d[far] < 1.8 * d[near]:
                return s.waypoint_xy(near), s.waypoint_xy(far)
    raise AssertionError("no such pair on the test floor")


@pytest.mark.parametrize("method", ("central_greedy", "central_hungarian"))
@pytest.mark.parametrize("fast_v, winner", [(1.0, 1), (0.5, 0)])
def test_a_central_dispatcher_prices_each_robot_at_its_own_speed(method, fast_v, winner):
    # robot_0 (0.5 m/s) is nearer the shelf than robot_1. At 1.0 m/s robot_1
    # gets there, and delivers, sooner; at the same 0.5 m/s it doesn't.
    s, gw = _gateway(method, [{"name": "standard"},
                              {"name": "fast", "count": 1, "max_speed_mps": fast_v}])
    assert [r.profile["type"] for r in s.robots] == ["standard", "fast"]
    near, far = _near_and_far(s, gw, "shelf_2_2")
    gw.on_status(0, 0.0, None, "idle", near)
    gw.on_status(1, 0.0, None, "idle", far)
    t = gw.request("shelf_2_2", "dock_2", 0.0)
    gw.tick(0.1)
    assert gw.tasks[t].robot == winner


@pytest.mark.parametrize("method", ("central_greedy", "central_hungarian"))
def test_a_central_dispatcher_gives_a_capability_task_only_to_a_capable_robot(method):
    s, gw = _gateway(method, HEAVY)
    k_heavy = next(r.k for r in s.robots if "heavy" in r.caps)
    gw.on_status(1 - k_heavy, 0.0, None, "idle", s.waypoint_xy("shelf_2_1"))
    gw.on_status(k_heavy, 0.0, None, "idle", s.waypoint_xy("dock_0"))
    t = gw.request("shelf_2_2", "dock_2", 0.0, requires=("heavy",))
    u = gw.request("shelf_2_0", "dock_2", 0.0)
    gw.tick(0.1)
    assert gw.tasks[t].robot == k_heavy and gw.tasks[u].robot == 1 - k_heavy
