"""scripts/mem_probe.py + scripts/mem_guard.py: attribution and trip rules."""
from __future__ import annotations

import json
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "scripts"))
sys.path.insert(0, str(REPO / "webui"))

import mem_guard  # noqa: E402
import mem_probe  # noqa: E402

GB = 2**30


def test_role_of_names_the_node_not_the_interpreter():
    r = mem_probe.role_of
    assert r(["python", "/repo/scripts/spawn_warehouse.py", "--headless"]) == "isaac"
    assert r(["/usr/bin/python3", "-u", "nodes/explorer.py", "--robot-id", "robot_1"]) == "explorer"
    # Extensionless entry points used to come out as "python3".
    assert r(["/usr/bin/python3", "/opt/ros/jazzy/bin/ros2", "launch", "x.py"]) == "ros2_launch"
    assert r(["python3", "/opt/ros/jazzy/lib/rosbridge_server/rosbridge_websocket",
              "--ros-args"]) == "rosbridge_websocket"
    assert r(["/opt/ros/jazzy/lib/rtabmap_slam/rtabmap", "--ros-args"]) == "rtabmap"


def test_sim_time_reads_the_padded_progress_line(tmp_path, monkeypatch):
    # spawn_warehouse.py prints t={:5.1f}: under 100 s there is a space after "=".
    log = tmp_path / "result.txt"
    log.write_text("  timeline.play()\n  t= 52.0s  robot_0 x=-4.5\n  t= 53.0s  robot_0 x=-4.6\n")
    monkeypatch.setattr(mem_probe, "SIM_LOG", log)
    assert mem_probe.sim_time() == 53.0
    log.write_text(log.read_text() + "  t=101.0s  robot_0\n")
    assert mem_probe.sim_time() == 101.0


def _sample(anon_gb=4.0, high_gb=16.0, avail_gb=20.0, psi_full=0.0, fence=True):
    s = {"sim_t": 100.0, "mem_avail_kb": int(avail_gb * 1048576), "swap_used_kb": 0,
         "psi": {"some": 0.0, "full": 0.0}, "gpu_mib": 3000, "cgroups": {},
         "groups_kb": {"shared/isaac": 6 * 1048576, "robot_0/rtabmap": 1048576}}
    if fence:
        s["cgroups"]["fence"] = {"anon": int(anon_gb * GB), "high": int(high_gb * GB),
                                 "max": int((high_gb + 4) * GB), "psi_full": psi_full}
    return s


def _args(**kw):
    a = mem_guard.parse_args([])
    a.min_avail_gb = 3.0
    for k, v in kw.items():
        setattr(a, k, v)
    return a


def test_guard_quiet_under_budget():
    assert mem_guard.verdict(_sample(anon_gb=10.0), _args()) is None


def test_guard_trips_on_the_fence_budget_not_page_cache():
    v = mem_guard.verdict(_sample(anon_gb=15.8), _args())
    assert v and "budget" in v
    # memory.current (with cache) isn't consulted: only anon counts.
    s = _sample(anon_gb=8.0)
    s["cgroups"]["fence"]["current"] = 30 * GB
    assert mem_guard.verdict(s, _args()) is None


def test_guard_counts_what_the_fence_swapped_out():
    # Throttled at MemoryHigh, anon stalls under the line and the rest swaps.
    s = _sample(anon_gb=14.0)
    assert mem_guard.verdict(s, _args()) is None
    s["cgroups"]["fence"]["swap"] = int(2.0 * GB)
    v = mem_guard.verdict(s, _args())
    assert v and "16.0 GB" in v


def test_guard_trips_on_machine_memory_even_without_a_fence():
    v = mem_guard.verdict(_sample(avail_gb=2.0, fence=False), _args())
    assert v and "MemAvailable" in v


def test_guard_trips_on_fence_pressure():
    v = mem_guard.verdict(_sample(psi_full=55.0), _args())
    assert v and "pressure" in v


def test_session_shows_the_guard_reason_only_for_this_run(tmp_path, monkeypatch):
    import session
    report = tmp_path / "guard.json"
    monkeypatch.setattr(session, "MEM_GUARD_REPORT", report)
    assert session.mem_guard_reason(0.0) is None
    report.write_text(json.dumps({"stopped_at": 1000.0, "reason": "sim memory 16 GB",
                                  "top": [["robot_0/rtabmap", 9000]]}))
    msg = session.mem_guard_reason(900.0)
    assert msg and "memory guard" in msg and "robot_0/rtabmap" in msg
    # A report older than the session belongs to an earlier run.
    assert session.mem_guard_reason(1500.0) is None
