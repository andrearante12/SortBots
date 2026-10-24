"""The comms overlay's engine: racks from an occupancy grid, the Hub fed the
way webui/operator_comms.js feeds it, and serve.py's endpoints for both.

Offline: PyYAML only, no ROS. The serve.py tests start a real serve.py on a
free port, as tests/splat_worker_test.py does.
"""
from __future__ import annotations

import json
import os
import queue
import socket
import subprocess
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path

import pytest

from commsim.des.layout import from_occupancy
from commsim.viz.demo import Hub

REPO = Path(__file__).resolve().parents[2]


def grid(w, h, cells):
    """Row-major, row 0 at the bottom; `cells` = occupied (x, y) cell coords."""
    occ = ["0"] * (w * h)
    for x, y in cells:
        occ[y * w + x] = "1"
    return "".join(occ)


# -- from_occupancy -----------------------------------------------------------

def test_a_wall_is_one_obstacle_however_thick():
    # A 2-cell-thick wall from y=1..6 at x=4..5, on a 10x8 grid of 0.5 m cells.
    occ = grid(10, 8, [(x, y) for x in (4, 5) for y in range(1, 7)])
    lay = from_occupancy(occ, 10, 8, 0.5)
    assert (lay.width, lay.height) == (5.0, 4.0)
    assert lay.racks == [(2.0, 0.5, 3.0, 3.5)]
    assert lay.crossings((0.5, 2.0), (4.5, 2.0)) == (1, 0)  # through it, once
    assert lay.crossings((0.5, 3.9), (4.5, 3.9)) == (0, 0)  # over the top of it


def test_an_l_shaped_blob_is_not_its_bounding_box():
    # The warehouse's outer wall is one blob whose box is the whole floor; a link
    # inside the room must not count as crossing it.
    border = [(x, 0) for x in range(10)] + [(0, y) for y in range(10)]
    lay = from_occupancy(grid(10, 10, border), 10, 10, 0.5)
    assert len(set(lay.groups)) == 1 and len(lay.racks) == 2
    assert lay.crossings((2.0, 2.0), (4.0, 4.0)) == (0, 0)
    assert lay.crossings((2.0, 2.0), (2.0, -0.1)) == (1, 0)


def test_separate_blobs_each_count_and_specks_are_dropped():
    occ = grid(12, 4, [(3, y) for y in range(4)] + [(7, y) for y in range(4)] + [(10, 1)])
    lay = from_occupancy(occ, 12, 4, 0.5, min_area_m2=0.5)
    assert len(set(lay.groups)) == 2          # the 1-cell speck at (10, 1) is gone
    assert lay.crossings((0.25, 1.0), (5.75, 1.0)) == (2, 0)


def test_bad_occupancy_size_is_refused():
    with pytest.raises(ValueError):
        from_occupancy("0101", 3, 3, 0.5)


# -- the Hub, fed like the overlay feeds it ------------------------------------------

def drain(q, seconds):
    end, out = time.monotonic() + seconds, []
    while time.monotonic() < end:
        try:
            out.append(json.loads(q.get(timeout=0.1)))
        except queue.Empty:
            pass
    return out


def test_overlay_sequence_routes_round_a_wall_via_the_gateway():
    hub = Hub()
    q = hub.subscribe()
    # 20 x 10 m floor, a wall down the middle with a gap at the top for the gateway.
    occ = grid(40, 20, [(x, y) for x in (19, 20) for y in range(0, 16)])
    for c in ({"op": "layout", "occ": occ, "w": 40, "h": 20, "cell": 0.5},
              {"op": "reset", "n": 2, "positions": [[10.0, 9.5], [6.0, 3.0], [14.0, 3.0]]},
              {"op": "range", "metres": 12}):
        hub.cmds.put(c)
    evs = drain(q, 2.5)
    init = [e for e in evs if e["k"] == "init"][-1]["header"]
    assert init["layout"]["name"] == "map" and init["n"] == 3
    assert [n["pos"] for n in init["nodes"]] == [[10.0, 9.5], [6.0, 3.0], [14.0, 3.0]]
    # The header carries the routes as they stand (the page clears its own on init).
    routes = {(n, d): nh for n, d, nh, _ in init["routes"]}
    assert routes[(1, 2)] == 0 and routes[(2, 1)] == 0  # robot 0 -> robot 1 through the AP
    assert any(e["k"] == "air" for e in evs)


def test_a_new_layout_keeps_the_model_and_its_gateway():
    hub = Hub()
    q = hub.subscribe()
    hub.cmds.put({"op": "layout", "occ": grid(20, 20, []), "w": 20, "h": 20, "cell": 0.5})
    hub.cmds.put({"op": "reset", "n": 1, "positions": [[2.0, 2.0], [8.0, 8.0]]})
    drain(q, 0.5)
    hub.cmds.put({"op": "gateway", "x": 3.0, "y": 4.0})
    hub.cmds.put({"op": "layout", "occ": grid(30, 20, [(15, 5)]), "w": 30, "h": 20, "cell": 0.5})
    evs = drain(q, 0.8)
    assert not [e for e in evs if e["k"] == "init"]                 # in place, not a reset
    assert [e for e in evs if e["k"] == "layout"][-1]["width"] == 15.0
    assert hub.sim.sim.nodes[0].pos == (3.0, 4.0)
    hub.cmds.put({"op": "reset", "n": 2, "positions": [[3.0, 4.0], [8.0, 8.0], [9.0, 8.0]]})
    drain(q, 0.5)
    assert hub.sim.layout_name == "map" and hub.sim.sim.layout.width == 15.0  # a reset keeps the map


def test_no_viewer_no_stepping():
    hub = Hub()
    t0 = hub.sim.sim.now
    time.sleep(0.6)
    assert hub.sim.sim.now == t0
    q = hub.subscribe()
    drain(q, 0.6)
    assert hub.sim.sim.now > t0


# -- serve.py ---------------------------------------------------------------------

def _free_port() -> int:
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


@pytest.fixture
def dashboard(tmp_path):
    port = _free_port()
    proc = subprocess.Popen(
        [sys.executable, str(REPO / "webui" / "serve.py"), "--platform", "sim",
         "--host", "127.0.0.1", "--port", str(port)],
        env={**os.environ, "SORTBOTS_COMMS_GATEWAY": str(tmp_path / "gw.json")},
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    base = f"http://127.0.0.1:{port}"
    for _ in range(100):
        try:
            urllib.request.urlopen(base + "/api/comms_gateway", timeout=1)
            break
        except OSError:
            time.sleep(0.05)
    yield base, tmp_path / "gw.json"
    proc.kill()
    proc.wait()


def _post(url, body):
    req = urllib.request.Request(url, data=json.dumps(body).encode(), method="POST",
                                 headers={"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=5) as r:
            return r.status, json.loads(r.read())
    except urllib.error.HTTPError as e:
        return e.code, json.loads(e.read())


def test_gateway_pin_round_trips_per_platform(dashboard):
    base, path = dashboard
    assert json.loads(urllib.request.urlopen(base + "/api/comms_gateway").read()) == \
        {"platform": "sim", "gateway": None}
    code, body = _post(base + "/api/comms_gateway", {"gateway": {"x": 1.23456, "y": -4}})
    assert code == 200 and body["gateway"] == {"x": 1.235, "y": -4.0}
    assert json.loads(path.read_text()) == {"sim": {"x": 1.235, "y": -4.0}}
    for bad in ({"x": "a", "y": 1}, {"x": float("1e9"), "y": 0}, [1, 2]):
        assert _post(base + "/api/comms_gateway", {"gateway": bad})[0] == 400
    assert _post(base + "/api/comms_gateway", {"gateway": None})[1]["gateway"] is None


def test_commsim_cmd_allows_only_overlay_ops_and_streams_events(dashboard):
    base, _ = dashboard
    # The overlay never posts tasks: the real fleet's auction is the task board.
    assert _post(base + "/api/commsim/cmd", {"op": "task"})[0] == 400
    assert _post(base + "/api/commsim/cmd", {"op": "rm -rf"})[0] == 400
    big = grid(400, 400, [])   # 160 k cells: over the 64 KB default body limit, under the overlay's
    assert _post(base + "/api/commsim/cmd", {"op": "layout", "occ": big, "w": 400, "h": 400,
                                              "cell": 0.5})[0] == 200
    with urllib.request.urlopen(base + "/api/commsim/events", timeout=5) as r:
        assert r.headers["Content-Type"] == "text/event-stream"
        line = r.readline().decode()
        assert line.startswith("data: ") and json.loads(line[6:])["k"] == "init"
