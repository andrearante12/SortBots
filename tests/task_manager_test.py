"""Offline tests for nodes/task_manager.py's queue and state machine.

task_manager imports rclpy / nav2_msgs / action_msgs at module scope and can't
be imported without a sourced ROS 2. These tests install minimal stand-ins in
sys.modules first, so they run under the system python3 with nothing sourced
(`/usr/bin/python3 -m pytest tests/`), and drive the node's callbacks by hand:
the fake action client records goals and the test decides how Nav2 answers.
"""
from __future__ import annotations

import json
import sys
import types
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parent.parent


# -- ROS stand-ins ---------------------------------------------------------------

class _Msg:
    def __init__(self, data=""):
        self.data = data


class _Logger:
    def info(self, *a, **k): pass
    def warn(self, *a, **k): pass
    def error(self, *a, **k): pass


class _Node:
    def __init__(self, name):
        self.subs, self.pubs = {}, {}

    def get_logger(self):
        return _Logger()

    def create_publisher(self, _type, topic, qos):
        pub = types.SimpleNamespace(sent=[], qos=qos)
        pub.publish = lambda m: pub.sent.append(json.loads(m.data) if topic.endswith("task_status") else m.data)
        self.pubs[topic] = pub
        return pub

    def create_subscription(self, _type, topic, cb, _qos):
        self.subs[topic] = cb

    def create_timer(self, *_a, **_k):
        pass

    def get_clock(self):
        return types.SimpleNamespace(now=lambda: types.SimpleNamespace(to_msg=lambda: None))


class _Future:
    def __init__(self):
        self.cbs, self.value = [], None

    def add_done_callback(self, cb):
        self.cbs.append(cb)

    def result(self):
        return self.value

    def resolve(self, value):
        self.value = value
        for cb in self.cbs:
            cb(self)


class _Handle:
    def __init__(self, accepted):
        self.accepted = accepted
        self.result_future = _Future()
        self.cancelled = False

    def get_result_async(self):
        return self.result_future

    def cancel_goal_async(self):
        self.cancelled = True


class _ActionClient:
    def __init__(self, *_a):
        self.ready = True
        self.goals = []   # [(goal, send_future)]

    def server_is_ready(self):
        return self.ready

    def wait_for_server(self, timeout_sec=0):
        return self.ready

    def send_goal_async(self, goal):
        f = _Future()
        self.goals.append((goal, f))
        return f


def _install_stubs():
    def mod(name, **attrs):
        m = types.ModuleType(name)
        m.__dict__.update(attrs)
        sys.modules[name] = m
        return m

    class _Pose:
        def __init__(self):
            self.header = types.SimpleNamespace(frame_id="", stamp=None)
            p = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)
            o = types.SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)
            self.pose = types.SimpleNamespace(position=p, orientation=o)

    class _Goal:
        def __init__(self):
            self.pose = None

    mod("rclpy", init=lambda: None, spin=lambda n: None, shutdown=lambda: None)
    mod("rclpy.node", Node=_Node)
    mod("rclpy.action", ActionClient=_ActionClient)
    mod("rclpy.qos", QoSProfile=lambda **kw: types.SimpleNamespace(**kw),
        QoSReliabilityPolicy=types.SimpleNamespace(RELIABLE="reliable"),
        QoSDurabilityPolicy=types.SimpleNamespace(TRANSIENT_LOCAL="transient_local"))
    mod("action_msgs")
    mod("action_msgs.msg", GoalStatus=types.SimpleNamespace(STATUS_SUCCEEDED=4))
    mod("geometry_msgs")
    mod("geometry_msgs.msg", PoseStamped=_Pose)
    mod("nav2_msgs")
    mod("nav2_msgs.action", NavigateToPose=types.SimpleNamespace(Goal=_Goal))
    mod("std_msgs")
    mod("std_msgs.msg", String=_Msg)


@pytest.fixture
def tm(tmp_path, monkeypatch):
    _install_stubs()
    wps = tmp_path / "nav_waypoints.json"
    wps.write_text(json.dumps({"sim": [
        {"name": "A", "x": 1, "y": 2, "yaw": 0},
        {"name": "B", "x": 3, "y": 4, "yaw": 0},
    ]}))
    monkeypatch.setenv("SORTBOTS_NAV_WAYPOINTS", str(wps))
    sys.path.insert(0, str(REPO_ROOT / "nodes"))
    sys.modules.pop("task_manager", None)
    import task_manager
    clock = {"t": 1000.0}
    monkeypatch.setattr(task_manager.time, "monotonic", lambda: clock["t"])
    node = task_manager.TaskManager("robot_0", "sim")
    node.clock, node.wps_file, node.mod = clock, wps, task_manager
    yield node
    sys.path.remove(str(REPO_ROOT / "nodes"))


def dispatch(node, **payload):
    node.subs["/robot_0/dispatch_task"](_Msg(json.dumps(payload)))


def statuses(node):
    return node.pubs["/robot_0/task_status"].sent


def answer(node, accepted=True, succeeded=None):
    """Nav2 answers the newest goal; succeeded=True/False also finishes it."""
    _goal, f = node.nav_client.goals[-1]
    h = _Handle(accepted)
    f.resolve(h)
    if accepted and succeeded is not None:
        h.result_future.resolve(types.SimpleNamespace(status=4 if succeeded else 6))
    return h


# -- tests ------------------------------------------------------------------------

def test_goto_runs_to_done(tm):
    dispatch(tm, goto="A", task_id="t1")
    assert statuses(tm)[-1]["queue"] == [{"task_id": "t1", "kind": "goto", "goto": "A"}]
    tm._tick()
    assert tm.state == "NAV_TO_GOAL"
    answer(tm, succeeded=True)
    assert [s["state"] for s in statuses(tm)[-2:]] == ["DONE", "IDLE"]
    assert statuses(tm)[-2]["task_id"] == "t1"


def test_route_runs_in_order(tm):
    for i, w in enumerate(["B", "A"]):
        dispatch(tm, goto=w, task_id=f"r{i}")
    tm._tick()
    assert tm.nav_client.goals[-1][0].pose.pose.position.x == 3.0  # B first
    answer(tm, succeeded=True)
    tm._tick()
    assert tm.current_task.task_id == "r1"


def test_unknown_waypoint_is_rejected_on_the_status_stream(tm):
    dispatch(tm, goto="Z", task_id="bad")
    s = statuses(tm)[-1]
    assert s["rejected"]["task_id"] == "bad" and "Z" in s["rejected"]["reason"]
    assert not tm.queue


def test_cancel_queued_and_running(tm):
    dispatch(tm, goto="A", task_id="run")
    dispatch(tm, goto="B", task_id="wait")
    tm._tick()
    h = answer(tm)            # accepted, still driving
    tm.subs["/robot_0/cancel_task"](_Msg("wait"))
    assert [t.task_id for t in tm.queue] == [] and tm.state == "NAV_TO_GOAL"
    tm.subs["/robot_0/cancel_task"](_Msg("run"))
    assert h.cancelled and tm.state == "IDLE" and statuses(tm)[-2]["state"] == "CANCELLED"
    # Nav2's late result for the cancelled goal must not touch whatever runs next.
    h.result_future.resolve(types.SimpleNamespace(status=6))
    assert statuses(tm)[-1]["state"] == "IDLE"


def test_rename_after_queueing_is_harmless(tm):
    dispatch(tm, goto="A", task_id="snap")
    tm.wps_file.write_text(json.dumps({"sim": [{"name": "Dock", "x": 1, "y": 2, "yaw": 0}]}))
    dispatch(tm, goto="Dock", task_id="new")  # reloads the set; "A" no longer exists
    tm._tick()                                 # must not KeyError on the stale name
    assert tm.current_task.task_id == "snap"


def test_waits_for_nav2_then_retries_rejections_during_startup(tm):
    tm.nav_client.ready = False
    dispatch(tm, goto="A", task_id="early")
    tm._tick()
    assert tm.state == "IDLE" and not tm.nav_client.goals   # held, not failed
    tm.nav_client.ready = True
    tm._tick()
    answer(tm, accepted=False)                 # server up but bt_navigator inactive
    assert tm.state == "IDLE" and [t.task_id for t in tm.queue] == ["early"]
    assert "FAILED" not in [s["state"] for s in statuses(tm)]
    tm._tick()
    assert len(tm.nav_client.goals) == 1       # backing off, no resend yet
    tm.clock["t"] += tm.mod.NAV_RETRY_SEC
    tm._tick()
    answer(tm, succeeded=True)
    assert statuses(tm)[-2] == {**statuses(tm)[-2], "task_id": "early", "state": "DONE"}


def test_rejection_after_the_grace_window_fails(tm):
    dispatch(tm, goto="A", task_id="late")
    tm._tick()
    tm.clock["t"] += tm.mod.NAV_STARTUP_GRACE_SEC + 1
    answer(tm, accepted=False)
    assert statuses(tm)[-2]["state"] == "FAILED"


def test_task_status_is_latched(tm):
    # fleet_tasks.py adopts what task_manager still holds after a restart of
    # its own: that only works if the last status reaches a late subscriber.
    qos = tm.pubs["/robot_0/task_status"].qos
    assert qos.durability == "transient_local" and qos.depth == 1


def test_carrying_task_skips_the_pickup_leg(tm):
    dispatch(tm, pickup="A", dropoff="B", task_id="fleet-7", carrying=True)
    assert statuses(tm)[-1]["queue"][0]["carrying"] is True
    tm._tick()
    assert tm.state == "NAV_TO_DROPOFF"
    assert tm.nav_client.goals[-1][0].pose.pose.position.x == 3.0  # B, not A


def test_exploration_resumes_only_after_the_task_that_stopped_it(tm):
    cmds = tm.pubs["/robot_0/explore_cmd"].sent
    explore = tm.subs["/robot_0/explore_status"]
    explore(_Msg(json.dumps({"state": "exploring"})))
    dispatch(tm, goto="A", task_id="t1")
    assert cmds == ["stop"]
    tm._tick()
    assert tm.state == "IDLE"   # explorer hasn't confirmed the stop yet
    explore(_Msg(json.dumps({"state": "stopped"})))
    tm._tick()
    answer(tm, succeeded=True)
    tm._tick()
    assert cmds == ["stop", "start"]
    tm._tick()
    assert cmds == ["stop", "start"]   # once, not every idle tick


def test_idle_never_starts_an_explorer_nobody_stopped(tm):
    dispatch(tm, goto="A", task_id="t1")
    tm._tick()
    answer(tm, succeeded=True)
    tm._tick()
    assert tm.pubs["/robot_0/explore_cmd"].sent == []


# -- package tasks: the magnet exchange ---------------------------------------------

PLACE = {"x": -8.02, "y": 9.9, "z": 0.8, "yaw": 1.5708}
CARGO = {"size": [0.24, 0.18, 0.12], "face": "+x", "at": {"x": -7.9, "y": 9.9}}


def package(tm, task_id="p1", pickup=None, dropoff=None, **over):
    dispatch(tm, task_id=task_id, package="pkg_0", pickup=pickup, dropoff=dropoff,
             place=PLACE, **CARGO, **over)


def requests(tm):
    return [json.loads(m) for m in tm.pubs["/robot_0/pick_request"].sent]


def result(tm, text):
    tm.subs["/robot_0/pick_result"](_Msg(text))


def test_package_task_without_legs_runs_with_no_nav2_at_all(tm):
    tm.nav_client.ready = False                    # Nav2 never came up: irrelevant to this task
    package(tm)
    assert statuses(tm)[-1]["queue"][0] == {"task_id": "p1", "kind": "pickup", "pickup": "here",
                                            "dropoff": "here", "package": "pkg_0"}
    tm._tick()
    assert tm.state == "PICKING" and tm.nav_client.goals == []
    assert requests(tm) == [{"cmd": "attach", "package": "pkg_0", "task_id": "p1", **CARGO}]
    result(tm, "picked")
    assert tm.state == "PLACING"                   # no dropoff leg: straight to the place
    assert requests(tm)[-1] == {"cmd": "detach", "package": "pkg_0", "task_id": "p1",
                                "place": PLACE, **CARGO}
    result(tm, "placed")
    assert [s["state"] for s in statuses(tm)[-2:]] == ["DONE", "IDLE"]
    assert statuses(tm)[-2]["task_id"] == "p1"


def test_package_task_with_legs_drives_between_the_two_exchanges(tm):
    package(tm, pickup={"name": "pose", "x": 5.0, "y": 6.0, "yaw": 1.0}, dropoff="B")
    tm._tick()
    assert tm.state == "NAV_TO_PICKUP"
    goal = tm.nav_client.goals[-1][0].pose.pose
    assert (goal.position.x, goal.position.y) == (5.0, 6.0)      # the inline pose, not a waypoint
    assert requests(tm) == []                                    # nothing asked of the arm yet
    answer(tm, succeeded=True)
    assert tm.state == "PICKING" and requests(tm)[-1]["cmd"] == "attach"
    result(tm, "picked")
    assert tm.state == "NAV_TO_DROPOFF"
    assert tm.nav_client.goals[-1][0].pose.pose.position.x == 3.0   # waypoint B
    answer(tm, succeeded=True)
    assert tm.state == "PLACING" and requests(tm)[-1]["cmd"] == "detach"
    result(tm, "placed")
    assert statuses(tm)[-2]["state"] == "DONE"


def test_a_failed_pick_fails_the_task_and_frees_the_queue(tm):
    package(tm, task_id="p1")
    package(tm, task_id="p2")
    tm._tick()
    result(tm, "failed")
    assert [s["state"] for s in statuses(tm)[-2:]] == ["FAILED", "IDLE"]
    assert statuses(tm)[-2]["task_id"] == "p1"
    tm._tick()
    assert tm.current_task.task_id == "p2" and requests(tm)[-1]["task_id"] == "p2"


def test_a_silent_controller_times_out(tm):
    package(tm)
    tm._tick()
    tm.clock["t"] += tm.mod.PICK_TIMEOUT_SEC - 1
    tm._tick()
    assert tm.state == "PICKING"
    tm.clock["t"] += 2
    tm._tick()
    assert statuses(tm)[-2]["state"] == "FAILED" and tm.state == "IDLE"
    assert requests(tm)[-1] == {"cmd": "cancel"}                 # arm stops, magnet off
    assert tm._pick_deadline is None


def test_results_out_of_turn_are_ignored(tm):
    result(tm, "picked")                                         # nothing running
    package(tm)
    tm._tick()
    result(tm, "placed")                                         # wrong state for it
    assert tm.state == "PICKING"
    result(tm, "picked")
    result(tm, "picked")                                         # a duplicate must not re-advance
    assert tm.state == "PLACING" and len(requests(tm)) == 2


def test_cancelling_mid_pick_tells_the_controller(tm):
    package(tm)
    tm._tick()
    tm.subs["/robot_0/cancel_task"](_Msg("p1"))
    assert requests(tm)[-1] == {"cmd": "cancel"}
    assert statuses(tm)[-2]["state"] == "CANCELLED" and tm.state == "IDLE"
    result(tm, "picked")                                         # its late answer is ignored
    assert tm.current_task is None


@pytest.mark.parametrize("payload", [
    {"package": "pkg_0", "place": PLACE, **{k: v for k, v in CARGO.items() if k != "face"}},
    {"package": "pkg_0", **CARGO},                                # no place
    {"package": "pkg_0", "place": {"x": 1}, **CARGO},
    {"package": "pkg_0", "place": PLACE, "size": [1, 2], "face": "+x", "at": CARGO["at"]},
    {"package": "pkg_0", "place": PLACE, **CARGO, "pickup": "Z"},      # unknown waypoint
    {"package": "pkg_0", "place": PLACE, **CARGO, "dropoff": {"x": 1}},  # pose with no y
])
def test_a_malformed_package_task_is_rejected_on_the_status_stream(tm, payload):
    dispatch(tm, task_id="bad", **payload)
    assert statuses(tm)[-1]["rejected"]["task_id"] == "bad"
    assert len(tm.queue) == 0


def test_the_seams_probe_to_task_manager_to_pick_controller(tm):
    """What the test layer sends, what task_manager makes of it and what the pick
    controller reads must be the same contract: each side is otherwise only tested
    against its own stand-in for the other."""
    sys.path.insert(0, str(REPO_ROOT))
    from simtest import spec
    from simtest.probe import dispatch_body
    from magnet_pick import parse_request

    resolved = spec.load("pick_place_smoke")["variants"][0][1]["tasks"][0]
    body = dispatch_body(dict(resolved, task_id="simtest_t0"))
    dispatch(tm, **body)
    assert "rejected" not in statuses(tm)[-1], statuses(tm)[-1]
    assert statuses(tm)[-1]["queue"][0]["package"] == "pkg_0"
    tm._tick()
    attach = requests(tm)[0]
    assert parse_request(json.dumps(attach)) == attach and attach["at"] == body["at"]
    result(tm, "picked")
    detach = requests(tm)[-1]
    assert parse_request(json.dumps(detach)) == detach and detach["place"] == body["place"]
    result(tm, "placed")
    assert statuses(tm)[-2]["state"] == "DONE" and statuses(tm)[-2]["task_id"] == "simtest_t0"


def test_a_pick_only_task_is_done_at_the_pick_and_never_lets_go(tm):
    dispatch(tm, task_id="p1", package="pkg_0", pickup=None, dropoff=None, place=None, **CARGO)
    assert "rejected" not in statuses(tm)[-1], statuses(tm)[-1]
    tm._tick()
    assert requests(tm)[-1]["cmd"] == "attach"
    result(tm, "picked")
    assert tm.state == "IDLE" and statuses(tm)[-2]["state"] == "DONE"
    assert [r["cmd"] for r in requests(tm)] == ["attach"]        # no detach, ever


def test_a_package_task_may_leave_the_location_to_the_robot(tm):
    dispatch(tm, task_id="p1", package="pkg_0", pickup=None, dropoff=None, place=PLACE, **{**CARGO, "at": None})
    assert "rejected" not in statuses(tm)[-1], statuses(tm)[-1]
    tm._tick()
    assert requests(tm)[-1]["at"] is None


def test_a_package_task_that_says_nothing_about_placing_is_refused(tm):
    dispatch(tm, task_id="p1", package="pkg_0", pickup=None, dropoff=None, **CARGO)
    assert "rejected" in statuses(tm)[-1]


def test_nav_only_dispatches_are_untouched_by_all_this(tm):
    dispatch(tm, pickup="A", dropoff="B", task_id="n1")
    assert "package" not in statuses(tm)[-1]["queue"][0]
    tm._tick()
    assert tm.state == "NAV_TO_PICKUP"
    answer(tm, succeeded=True)
    assert tm.state == "PICKING" and requests(tm) == []          # a dwell, no pick_request
    tm.clock["t"] += tm.mod.DWELL_SEC + 0.1
    tm._tick()
    assert tm.state == "NAV_TO_DROPOFF"
