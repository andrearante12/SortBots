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
