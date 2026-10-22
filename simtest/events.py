"""The event stream metrics consume — the seam between ROS and scoring.

Metrics never subscribe to ROS. simtest/probe.py turns topics into plain
events (dicts), and the same events are written to the run's events.jsonl.gz.
That is what makes a metric's definition editable after the fact: change how
something is calculated, bump the metric's version, and
`scripts/simtest.py rescore RUN_DIR` replays the recorded stream through the
new code — no sim, no GPU, identical inputs.

Every event is {"s": STREAM, "t": SECONDS, ...payload}. `t` is SIM seconds
since the test's t0 (when the probe restarted the world clock — see
simtest/world.py "ACTOR TIME"), so actor poses, goal timings and every
metric share one clock. Events before t0 are never delivered.

STREAMS documents each one. A metric declares the streams it reads
(Metric.streams); the probe subscribes to their union, plus the always-on
control streams, and records everything it delivers. The QoS of every topic
behind a stream is chosen HERE, centrally, rather than by each metric — the
invariant that matters (CLAUDE.md: a RELIABLE image subscriber kills Isaac's
image writers) is then enforced in one place: no stream is an image.
"""
from __future__ import annotations

import base64
import gzip
import json
import zlib
from pathlib import Path

STREAMS = {
    "start": "t0 marker: {wall, sim} — once, first event of every run",
    "end": "last event: {reason, sim_s, wall_s}",
    "gt": "/sim/ground_truth, 10 Hz: {wt, robots: {rid: {x,y,yaw}}, "
          "actors: {id: {x,y,yaw,vx,vy,moving}}}",
    "goal": "probe-sent Nav2 goals: {rid, goal_id, state: sent|accepted|rejected|"
            "succeeded|aborted|canceled|timeout, pose: [x,y,yaw]}",
    "dispatch": "probe-sent task_manager tasks: {rid, task_id, pickup, dropoff}",
    "task": "/<rid>/task_status JSON: {rid, task_id, state, ...}",
    "explore": "/<rid>/explore_status JSON (throttled to 1 Hz): {rid, state, ...}",
    "map": "/map OccupancyGrid, throttled (default every 5 sim-s, plus the last): "
           "{grid: MapGrid}",
    "dyn": "/<rid>/dynamic_obstacles in the map frame, <=5 Hz, 0.1 m voxels: "
           "{rid, pts: [[x, y], ...]}",
}

# Recorded on every run whether or not a metric asks, so a NEW metric can be
# scored against old runs. `dyn` is opt-in (metric or `record:`) — it is the
# only stream whose size scales with the scene.
ALWAYS_RECORDED = frozenset({"start", "end", "gt", "goal", "dispatch", "task",
                             "explore", "map"})


class MapGrid:
    """An OccupancyGrid's data as int8 numpy, row 0 at origin y (ROS order)."""

    __slots__ = ("width", "height", "resolution", "ox", "oy", "data")

    def __init__(self, width, height, resolution, ox, oy, data):
        import numpy as np

        self.width, self.height = int(width), int(height)
        self.resolution, self.ox, self.oy = float(resolution), float(ox), float(oy)
        self.data = np.asarray(data, dtype=np.int8).reshape(self.height, self.width)

    def key(self) -> tuple:
        return (self.width, self.height, round(self.resolution, 6),
                round(self.ox, 4), round(self.oy, 4))

    def cell_centres(self):
        """(xs, ys) world coordinates of every cell centre, each (h, w)."""
        import numpy as np

        xs = self.ox + (np.arange(self.width) + 0.5) * self.resolution
        ys = self.oy + (np.arange(self.height) + 0.5) * self.resolution
        return np.meshgrid(xs, ys)

    def to_json(self) -> dict:
        raw = self.data.astype("int8").tobytes()
        return {"w": self.width, "h": self.height, "res": self.resolution,
                "ox": self.ox, "oy": self.oy,
                "z": base64.b64encode(zlib.compress(raw, 6)).decode("ascii")}

    @classmethod
    def from_json(cls, obj: dict) -> "MapGrid":
        import numpy as np

        raw = zlib.decompress(base64.b64decode(obj["z"]))
        return cls(obj["w"], obj["h"], obj["res"], obj["ox"], obj["oy"],
                   np.frombuffer(raw, dtype=np.int8))


def encode(ev: dict) -> str:
    if ev.get("s") == "map":
        ev = {**ev, "grid": ev["grid"].to_json()}
    return json.dumps(ev, separators=(",", ":"))


def decode(line: str) -> dict:
    ev = json.loads(line)
    if ev.get("s") == "map":
        ev["grid"] = MapGrid.from_json(ev["grid"])
    return ev


class Recorder:
    """Append-only events.jsonl.gz writer."""

    def __init__(self, path: Path):
        self.path = Path(path)
        self._fh = gzip.open(self.path, "wt", encoding="utf-8")
        self.count = 0

    def write(self, ev: dict) -> None:
        self._fh.write(encode(ev) + "\n")
        self.count += 1

    def close(self) -> None:
        self._fh.close()


def read(path: Path):
    """Yield the events of a recorded run, in order."""
    with gzip.open(path, "rt", encoding="utf-8") as fh:
        for line in fh:
            if line.strip():
                yield decode(line)


def recorded_streams(path: Path) -> set[str]:
    return {ev["s"] for ev in read(path)}
