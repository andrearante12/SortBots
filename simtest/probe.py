#!/usr/bin/env python3
"""Drive and observe one live sim run for a test case, then score it.

Started by scripts/simtest.py inside a ROS-sourced subshell (scripts/ros_bash.sh
when present, else plain bash), never by hand in the shell that launched Isaac:

    python3 -m simtest.probe --variant RUN_DIR/variant.json --out RUN_DIR

Everything it decides is in ProbeCore below, which is pure (no rclpy) and
unit-tested with a fake clock; the Probe node is only the wiring. Phases:

  ready    /clock, /sim/ground_truth and the first /map have arrived, and every
           robot with a goal has its navigate_to_pose server up
  warmup   `warmup_s` sim seconds more, so costmaps and the first SLAM
           keyframes exist before anything is asked of the robot
  t0       publish /sim/world_cmd "restart"; t0 is the sim time the world
           clock restarted at, read back from ground truth (gt.t - gt.wt) —
           so actors and goals share t=0 to within one ground-truth tick
  running  goals/tasks sent on schedule, events recorded and fed to metrics
  settle   an `until` condition held; keep scoring `settle_s` more
  done     final map emitted, `end` event, score written

QoS, because getting it wrong here hurts the run being measured: no image
topic is ever subscribed (CLAUDE.md — a RELIABLE image subscriber kills
Isaac's image writers); /clock and the depth-derived clouds are best-effort
to match their publishers; /map is transient-local so a late start still
gets the current grid.

Exit codes: 0 the run completed and was scored (pass OR fail — the verdict
is in score.json), 2 infrastructure (stack never ready, wall timeout),
130 interrupted.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from simtest import events as ev_lib  # noqa: E402
from simtest import metrics as metrics_lib  # noqa: E402

MAP_EVERY_S = 5.0       # sim seconds between recorded /map samples
EXPLORE_EVERY_S = 1.0
DYN_EVERY_S = 0.2       # 5 Hz per robot
DYN_VOXEL_M = 0.1
DYN_MAX_POINTS = 300
GOAL_TERMINAL = {"succeeded", "aborted", "rejected", "canceled", "timeout"}


class ProbeCore:
    """Every decision the probe makes, minus ROS. Times are SIM seconds."""

    def __init__(self, resolved: dict, streams: set[str], *, recorder=None):
        self.r = resolved
        self.streams = set(streams)
        self.recorder = recorder
        self.ms = metrics_lib.MetricSet(resolved["metrics"], self._ctx())
        self.phase = "ready"
        self.sim = None                     # latest /clock
        self.wall0 = None
        self.ready_at = None                # sim time readiness was reached
        self.restart_sent_at = None
        self.t0 = None
        self.have = {"gt": False, "map": False}
        self.nav_ready: set[str] = set()
        self.goals = {g["id"]: {**g, "state": None, "sent_at": None, "ended_at": None}
                      for g in resolved["goals"]}
        self.tasks = {t["id"]: {**t, "state": None, "sent_at": None}
                      for t in resolved["tasks"]}
        self.explore: dict[str, str] = {}
        self.until_met_at = None
        self.reason = None
        self._last = {}                     # throttle bookkeeping: key -> t
        self._pending_map = None

    def _ctx(self):
        from simtest import spec

        return spec.context(self.r)

    # -- time ------------------------------------------------------------------
    def elapsed(self) -> float | None:
        return None if self.t0 is None or self.sim is None else self.sim - self.t0

    def emit(self, ev: dict) -> None:
        if self.t0 is None:
            return
        ev.setdefault("t", round(self.elapsed(), 4))
        if ev["s"] in self.streams:
            if self.recorder is not None:
                self.recorder.write(ev)
            self.ms.feed(ev)

    def throttled(self, key: str, every: float) -> bool:
        el = self.elapsed()
        if el is None:
            return True
        last = self._last.get(key)
        if last is not None and el - last < every:
            return True
        self._last[key] = el
        return False

    # -- inputs ----------------------------------------------------------------
    def on_clock(self, sim: float) -> None:
        self.sim = sim

    def on_gt(self, gt: dict) -> None:
        self.have["gt"] = True
        # Ground truth is stamped in sim time too; never let the clock lag it.
        self.sim = gt["t"] if self.sim is None else max(self.sim, gt["t"])
        # The restart has landed when the world clock reads no more than the
        # sim time since we asked (+1 s for /clock vs ground-truth stamping
        # jitter). Before it lands, an autostarted clock reads ~the whole run.
        if self.t0 is None and self.restart_sent_at is not None and gt.get("running") \
                and gt["t"] >= self.restart_sent_at - 1.0 \
                and gt["wt"] <= gt["t"] - self.restart_sent_at + 1.0:
            self.t0 = gt["t"] - gt["wt"]
            self.phase = "running"
            self.emit({"s": "start", "t": 0.0, "sim": round(self.t0, 4),
                       "wall": round(time.monotonic() - (self.wall0 or time.monotonic()), 2)})
        if self.t0 is not None:
            self.emit({"s": "gt", "t": round(gt["t"] - self.t0, 4), "wt": gt["wt"],
                       "robots": gt["robots"], "actors": gt["actors"]})

    def on_map(self, grid) -> None:
        self.have["map"] = True
        self._pending_map = grid
        if not self.throttled("map", MAP_EVERY_S):
            self.emit({"s": "map", "grid": grid})
            self._pending_map = None

    def on_task(self, rid: str, obj: dict) -> None:
        tid = obj.get("task_id")
        for t in self.tasks.values():
            if t["robot"] == rid and t.get("task_id") == tid:
                t["state"] = obj.get("state")
        self.emit({"s": "task", "rid": rid, **{k: v for k, v in obj.items() if k != "s"}})

    def on_explore(self, rid: str, obj: dict) -> None:
        changed = self.explore.get(rid) != obj.get("state")
        self.explore[rid] = obj.get("state")
        if changed or not self.throttled(f"explore:{rid}", EXPLORE_EVERY_S):
            keep = {k: obj.get(k) for k in ("state", "goals_sent", "goals_succeeded",
                                            "goals_failed", "free_area_m2")}
            self.emit({"s": "explore", "rid": rid, **keep})

    def on_goal_state(self, gid: str, state: str) -> None:
        g = self.goals[gid]
        if g["state"] in GOAL_TERMINAL:
            return  # e.g. the result of a goal we already timed out and cancelled
        g["state"] = state
        if state in GOAL_TERMINAL:
            g["ended_at"] = self.elapsed()
        self.emit({"s": "goal", "rid": g["robot"], "goal_id": gid, "state": state,
                   "pose": g["pose"]})

    # -- decisions -------------------------------------------------------------
    def missing(self) -> list[str]:
        """What readiness is still waiting for — logged while waiting, because
        "phase ready" for ten minutes says nothing about which input is late."""
        out = [] if self.sim is not None else ["/clock"]
        if not self.have["gt"]:
            out.append("/sim/ground_truth")
        if not self.have["map"] and "map" in self.streams:
            out.append("/map")
        out += [f"/{r}/navigate_to_pose" for r in sorted({g["robot"] for g in self.goals.values()})
                if r not in self.nav_ready]
        return out

    def ready(self) -> bool:
        need_nav = {g["robot"] for g in self.goals.values()}
        return (self.sim is not None and self.have["gt"]
                and (self.have["map"] or "map" not in self.streams)
                and need_nav <= self.nav_ready)

    def want_restart(self) -> bool:
        """True exactly once: when it is time to send the world-clock restart."""
        if self.restart_sent_at is not None or self.sim is None:
            return False
        if self.ready_at is None:
            if not self.ready():
                return False
            self.ready_at = self.sim
            self.phase = "warmup"
        if self.sim - self.ready_at >= self.r["warmup_s"]:
            self.restart_sent_at = self.sim
            self.phase = "t0"
            return True
        return False

    def due_goals(self) -> list[dict]:
        """Goals to send now: at_s reached and the robot's previous goal ended."""
        el = self.elapsed()
        if el is None:
            return []
        out, busy = [], set()
        for g in self.goals.values():          # dict order == the test's order
            rid = g["robot"]
            if rid in busy:
                continue
            if g["state"] is None:
                if el >= g["at_s"]:
                    g["state"] = "sending"
                    g["sent_at"] = el
                    out.append(g)
                busy.add(rid)
            elif g["state"] not in GOAL_TERMINAL:
                busy.add(rid)
        return out

    def timed_out_goals(self) -> list[dict]:
        el = self.elapsed()
        return [g for g in self.goals.values()
                if el is not None and g["sent_at"] is not None
                and g["state"] not in GOAL_TERMINAL and el - g["sent_at"] > g["timeout_s"]]

    def due_tasks(self) -> list[dict]:
        el = self.elapsed()
        out = []
        for t in self.tasks.values():
            if el is not None and t["sent_at"] is None and el >= t["at_s"]:
                t["sent_at"] = el
                t["task_id"] = f"simtest_{t['id']}"
                out.append(t)
        return out

    def stop_reason(self) -> str | None:
        el = self.elapsed()
        if el is None:
            return None
        if el >= self.r["stop"]["sim_s"]:
            return "sim_s"
        until = self.r["stop"]["until"]
        met = None
        if "goals_done" in until and self.goals and \
                all(g["state"] in GOAL_TERMINAL for g in self.goals.values()):
            met = "goals_done"
        if "tasks_done" in until and self.tasks and \
                all(t["state"] in ("DONE", "FAILED") for t in self.tasks.values()):
            met = met or "tasks_done"
        if "explorer_done" in until and self.r["robots"] and \
                all(self.explore.get(r) == "done" for r in self.r["robots"]):
            met = met or "explorer_done"
        if met is None:
            self.until_met_at = None
            return None
        if self.until_met_at is None:
            self.until_met_at = el
            self.phase = "settle"
        return met if el - self.until_met_at >= self.r["stop"]["settle_s"] else None

    def finish(self, reason: str, wall_s: float) -> dict:
        self.reason = reason
        if self._pending_map is not None:
            self.emit({"s": "map", "grid": self._pending_map})
        el = self.elapsed()
        self.emit({"s": "end", "reason": reason, "sim_s": round(el or 0.0, 3),
                   "wall_s": round(wall_s, 2)})
        self.phase = "done"
        return self.ms.score()


# ---------------------------------------------------------------------------
# ROS wiring
# ---------------------------------------------------------------------------

def _voxel_xy(pts, size: float, cap: int) -> list[list[float]]:
    import numpy as np

    if len(pts) == 0:
        return []
    keys = np.floor(pts[:, :2] / size).astype(np.int64)
    _, idx = np.unique(keys, axis=0, return_index=True)
    sel = pts[np.sort(idx)][:cap, :2]
    return [[round(float(x), 3), round(float(y), 3)] for x, y in sel]


def run(variant_path: Path, out_dir: Path, startup_timeout: float) -> int:
    import functools

    import numpy as np
    import rclpy
    from action_msgs.msg import GoalStatus
    from geometry_msgs.msg import PoseStamped
    from nav2_msgs.action import NavigateToPose
    from nav_msgs.msg import OccupancyGrid
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                           qos_profile_sensor_data)
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import PointCloud2
    from std_msgs.msg import String

    variant = json.loads(Path(variant_path).read_text())
    resolved, streams = variant["resolved"], set(variant["streams"])
    recorder = ev_lib.Recorder(out_dir / "events.jsonl.gz")
    core = ProbeCore(resolved, streams, recorder=recorder)
    core.wall0 = time.monotonic()
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
    status_names = {GoalStatus.STATUS_SUCCEEDED: "succeeded",
                    GoalStatus.STATUS_ABORTED: "aborted",
                    GoalStatus.STATUS_CANCELED: "canceled"}

    def log(msg: str) -> None:
        print(f"[probe] {msg}", flush=True)

    class Probe(Node):
        def __init__(self):
            super().__init__("simtest_probe")
            self.create_subscription(Clock, "/clock", self._on_clock, qos_profile_sensor_data)
            self.create_subscription(String, "/sim/ground_truth", self._on_gt, 50)
            self.world_cmd = self.create_publisher(String, "/sim/world_cmd", 10)
            if "map" in streams:
                self.create_subscription(OccupancyGrid, "/map", self._on_map, latched)
            self.nav, self.handles, self.dispatch = {}, {}, {}
            for rid in resolved["robots"]:
                # functools.partial, never `lambda m, r=rid:` — rclpy counts the
                # callback's parameters to decide whether to also pass
                # MessageInfo, and would hand it in as `r`.
                self.create_subscription(String, f"/{rid}/task_status",
                                         functools.partial(self._on_json, "task", rid), 10)
                self.create_subscription(String, f"/{rid}/explore_status",
                                         functools.partial(self._on_json, "explore", rid), 10)
                if any(g["robot"] == rid for g in resolved["goals"]):
                    self.nav[rid] = ActionClient(self, NavigateToPose, f"/{rid}/navigate_to_pose")
                if any(t["robot"] == rid for t in resolved["tasks"]):
                    self.dispatch[rid] = self.create_publisher(String, f"/{rid}/dispatch_task", 10)
                if "dyn" in streams:
                    self.create_subscription(PointCloud2, f"/{rid}/dynamic_obstacles",
                                             functools.partial(self._on_dyn, rid),
                                             qos_profile_sensor_data)
            self.tf = None
            if "dyn" in streams:
                import tf2_ros

                self.tf_buffer = tf2_ros.Buffer()
                self.tf = tf2_ros.TransformListener(self.tf_buffer, self)

        def _on_clock(self, m):  # not _clock: rclpy Node owns that name
            core.on_clock(m.clock.sec + m.clock.nanosec * 1e-9)

        def _on_gt(self, m):
            try:
                core.on_gt(json.loads(m.data))
            except (ValueError, KeyError) as e:
                log(f"bad ground truth: {e}")

        def _on_map(self, m):
            from simtest.events import MapGrid

            o = m.info.origin.position
            core.on_map(MapGrid(m.info.width, m.info.height, m.info.resolution, o.x, o.y,
                                np.asarray(m.data, dtype=np.int8)))

        def _on_json(self, kind, rid, m):
            try:
                obj = json.loads(m.data)
            except ValueError:
                return
            if not isinstance(obj, dict):
                return
            (core.on_task if kind == "task" else core.on_explore)(rid, obj)

        def _on_dyn(self, rid, m):
            if core.t0 is None or core.throttled(f"dyn:{rid}", DYN_EVERY_S):
                return
            from rclpy.time import Time
            from sensor_msgs_py import point_cloud2

            pts = point_cloud2.read_points_numpy(m, field_names=("x", "y", "z"),
                                                 skip_nans=True)
            xy = []
            if len(pts):
                try:
                    tr = self.tf_buffer.lookup_transform("map", m.header.frame_id,
                                                         Time.from_msg(m.header.stamp))
                except Exception:  # noqa: BLE001 — extrapolation etc.; use the latest
                    try:
                        tr = self.tf_buffer.lookup_transform("map", m.header.frame_id, Time())
                    except Exception:  # noqa: BLE001
                        return
                q = tr.transform.rotation
                t = tr.transform.translation
                x, y, z, w = q.x, q.y, q.z, q.w
                R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                              [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                              [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
                world = pts.astype(np.float64) @ R.T + np.array([t.x, t.y, t.z])
                xy = _voxel_xy(world, DYN_VOXEL_M, DYN_MAX_POINTS)
            core.emit({"s": "dyn", "rid": rid, "pts": xy})

        # -- goals ------------------------------------------------------------
        def send_goal(self, g):
            ps = PoseStamped()
            ps.header.frame_id = "map"
            ps.pose.position.x, ps.pose.position.y = g["pose"][0], g["pose"][1]
            ps.pose.orientation.z = math.sin(g["pose"][2] / 2.0)
            ps.pose.orientation.w = math.cos(g["pose"][2] / 2.0)
            goal = NavigateToPose.Goal(pose=ps)
            core.on_goal_state(g["id"], "sent")
            log(f"goal {g['id']} -> {g['robot']} {g['pose']}")
            fut = self.nav[g["robot"]].send_goal_async(goal)
            fut.add_done_callback(functools.partial(self._on_goal_response, g["id"]))

        def _on_goal_response(self, gid, fut):
            handle = fut.result()
            if handle is None or not handle.accepted:
                core.on_goal_state(gid, "rejected")
                log(f"goal {gid} rejected")
                return
            self.handles[gid] = handle
            core.on_goal_state(gid, "accepted")
            handle.get_result_async().add_done_callback(
                functools.partial(self._on_goal_result, gid))

        def _on_goal_result(self, gid, fut):
            status = fut.result().status
            state = status_names.get(status, "aborted")
            core.on_goal_state(gid, state)
            log(f"goal {gid} {state} at t={core.elapsed():.1f}s")

        def timeout_goal(self, g):
            h = self.handles.get(g["id"])
            core.on_goal_state(g["id"], "timeout")
            log(f"goal {g['id']} timed out after {g['timeout_s']:.0f}s — cancelling")
            if h is not None:
                h.cancel_goal_async()

        def step(self):
            for rid, client in self.nav.items():
                if rid not in core.nav_ready and client.server_is_ready():
                    core.nav_ready.add(rid)
            if core.want_restart():
                self.world_cmd.publish(String(data=json.dumps({"cmd": "restart"})))
                log(f"stack ready + {resolved['warmup_s']:.0f}s warmup: restarting the "
                    f"world clock at sim t={core.sim:.1f}")
            for g in core.due_goals():
                self.send_goal(g)
            for g in core.timed_out_goals():
                self.timeout_goal(g)
            for t in core.due_tasks():
                body = {"pickup": t["pickup"], "dropoff": t["dropoff"], "task_id": t["task_id"]}
                self.dispatch[t["robot"]].publish(String(data=json.dumps(body)))
                core.emit({"s": "dispatch", "rid": t["robot"], "task_id": t["task_id"],
                           "pickup": t["pickup"], "dropoff": t["dropoff"]})
                log(f"task {t['task_id']} -> {t['robot']}")

    rclpy.init()
    node = Probe()
    cap_s = resolved["stop"]["sim_s"] + resolved["warmup_s"]
    deadline = time.monotonic() + startup_timeout + cap_s / 0.1  # a pessimistic 0.1x RTF
    reason, last_beat, last_phase = None, 0.0, None
    try:
        while reason is None:
            rclpy.spin_once(node, timeout_sec=0.05)
            node.step()
            reason = core.stop_reason()
            now = time.monotonic()
            if core.phase != last_phase:
                log(f"phase {core.phase}")
                last_phase = core.phase
            if core.t0 is None and now - last_beat > 20:
                last_beat = now
                missing = core.missing()
                log(f"waiting for: {', '.join(missing)}" if missing else
                    f"{core.phase}: sim t={core.sim:.1f}")
            if core.t0 is None and now - core.wall0 > startup_timeout:
                log(f"never ready after {startup_timeout:.0f}s wall "
                    f"(missing: {', '.join(core.missing()) or 'nothing — world restart not seen'})")
                reason = "not_ready"
            elif now > deadline:
                reason = "wall_timeout"
            if core.t0 is not None and now - last_beat > 15:
                last_beat = now
                parts = [f"{g['id']}={g['state']}" for g in core.goals.values()]
                parts += [f"{t['id']}={t['state']}" for t in core.tasks.values()]
                parts += [f"{r}:{s}" for r, s in sorted(core.explore.items())]
                log(f"t={core.elapsed():6.1f}s  " + (", ".join(parts) or "running"))
    except KeyboardInterrupt:
        reason = "interrupted"
    wall_s = time.monotonic() - core.wall0
    score = core.finish(reason, wall_s)
    recorder.close()
    meta = {"stop_reason": reason, "sim_s": core.elapsed(), "wall_s": round(wall_s, 1),
            "t0_sim": core.t0, "events": recorder.count}
    (out_dir / "score.json").write_text(json.dumps({"score": score, "meta": meta}, indent=2,
                                                   default=str))
    log(f"stop: {reason}  t={meta['sim_s']}  verdict={'PASS' if score['ok'] else 'FAIL'}")
    try:
        node.destroy_node()
        rclpy.shutdown()
    except Exception:  # noqa: BLE001 — already shutting down after Ctrl-C
        pass
    if reason in ("not_ready", "wall_timeout"):
        return 2
    return 130 if reason == "interrupted" else 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--variant", required=True, help="variant.json written by simtest.py")
    ap.add_argument("--out", required=True, help="the run directory")
    ap.add_argument("--startup-timeout", type=float, default=900.0,
                    help="wall seconds to wait for the stack to be ready")
    a = ap.parse_args(argv)
    return run(Path(a.variant), Path(a.out), a.startup_timeout)


if __name__ == "__main__":
    sys.exit(main())
