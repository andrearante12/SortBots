#!/usr/bin/env python3
"""Where the sim's memory goes: per robot, per process role, over sim time.

    scripts/mem_probe.py record --out data/memprobe/<name> [--interval 10]
    scripts/mem_probe.py summarize data/memprobe/<name> [more dirs...]

`record` samples until the pipeline (spawn_warehouse.py) has been seen and is
gone again, or until Ctrl-C. Each sample is one JSON line in samples.jsonl:
system MemAvailable / swap / memory PSI, GPU memory, the two cgroups the
pipeline lives in, and the PSS of every pipeline process grouped by
(robot, role). PSS, not RSS: a dozen ROS nodes per robot map the same
rclpy/DDS libraries, and RSS counts those once per process — summing RSS
over-reported a 2-robot stack by GBs.

Robot comes from a `robot_<n>` token in the command line (every per-robot
node gets --robot-id or a /robot_<n> namespace); anything without one is
"shared" (Isaac, map_merge, the console). Role is the node's script or
executable name.

Sim time is the last `t=<s>` progress line spawn_warehouse.py emits, so growth reads per
sim-second without ROS: a slow-RTF run maps less per wall-minute, and a
wall-clock slope would blame that on memory.

System python3 only, no dependencies — it has to run next to a sim that is
already short of memory.
"""
from __future__ import annotations

import argparse
import json
import os
import re
import signal
import subprocess
import sys
import time
from pathlib import Path

# spawn_warehouse.py's emit() file: the same progress lines as its stdout, but
# Kit can swallow late stdout writes and this file it can't.
SIM_LOG = Path(os.environ.get("ISAAC_SPAWN_WAREHOUSE_RESULT",
                              "/tmp/isaac_spawn_warehouse_result.txt"))
CGROUP_ROOT = Path("/sys/fs/cgroup")
ROBOT_RE = re.compile(r"\brobot_(\d+)\b")
SIM_T_RE = re.compile(rb"\bt=\s*(\d+(?:\.\d+)?)s\b")  # "t= 52.0s": %5.1f pads

# Command-line fragments that mark a process as part of the sim pipeline or the
# console. Mirrors run_demo.sh's PIPELINE_PATTERNS / CONSOLE_PATTERNS loosely:
# a miss here only means a process is unattributed, never that it is killed.
PIPELINE_MARKS = (
    "spawn_warehouse.py", "rtabmap", "nav2_", "/opt/ros/jazzy/lib/nav2",
    "component_container", "controller_server", "planner_server", "bt_navigator",
    "behavior_server", "smoother_server", "velocity_smoother", "collision_monitor",
    "lifecycle_manager", "waypoint_follower", "map_server",
    "/nodes/", "static_transform_publisher", "ros2 launch", "ros2launch",
    "rosbridge", "web_video_server", "webui/serve.py", "robot_state_publisher",
)


def meminfo() -> dict:
    out = {}
    with open("/proc/meminfo") as f:
        for line in f:
            k, v = line.split(":", 1)
            out[k] = int(v.split()[0])  # kB
    return out


def psi_memory() -> dict:
    """{'some': avg10, 'full': avg10} from /proc/pressure/memory."""
    out = {}
    try:
        for line in Path("/proc/pressure/memory").read_text().splitlines():
            kind, *fields = line.split()
            out[kind] = float(dict(f.split("=") for f in fields)["avg10"])
    except OSError:
        pass
    return out


def gpu_used_mib() -> int | None:
    try:
        r = subprocess.run(
            ["nvidia-smi", "--query-gpu=memory.used,memory.total",
             "--format=csv,noheader,nounits"],
            capture_output=True, text=True, timeout=5)
        used, total = (int(x) for x in r.stdout.split(",")[:2])
        return used
    except (OSError, ValueError, subprocess.TimeoutExpired):
        return None


def sim_time() -> float | None:
    """Last `t=<s>` in Isaac's log; reads only the tail, the log runs to MBs."""
    try:
        with open(SIM_LOG, "rb") as f:
            f.seek(0, os.SEEK_END)
            f.seek(max(0, f.tell() - 16384))
            hits = SIM_T_RE.findall(f.read())
        return float(hits[-1]) if hits else None
    except OSError:
        return None


def _user_cgroup() -> Path:
    return CGROUP_ROOT / f"user.slice/user-{os.getuid()}.slice/user@{os.getuid()}.service"


def fence_cgroup() -> Path | None:
    """run_demo.sh's resource fence: the sortbots-sim-<pid> systemd scope the
    whole pipeline runs in. Isaac is started inside it, and ros_bash.sh moves
    each in-box (distrobox) process into it before it forks, because `podman
    exec` would otherwise start it in the box's own libpod-*.scope (fixed
    2026-10-19, after rtabmap hit 13 GB outside the fence)."""
    hits = sorted(_user_cgroup().glob("*/sortbots-sim-*.scope"))
    return hits[-1] if hits else None


def cgroup_stats(cg: Path) -> dict:
    """Bytes, plus the limits (None = unlimited) and the cgroup's own memory
    PSI. `anon` is what can't be dropped: page cache in `current` is
    reclaimable, so a budget check on `current` would trip on file I/O."""
    out: dict = {}
    try:
        out["current"] = int((cg / "memory.current").read_text())
        for line in (cg / "memory.stat").read_text().splitlines():
            k, v = line.split()
            if k in ("anon", "file", "shmem"):
                out[k] = int(v)
        for k in ("high", "max"):
            v = (cg / f"memory.{k}").read_text().strip()
            out[k] = None if v == "max" else int(v)
        for line in (cg / "memory.pressure").read_text().splitlines():
            kind, *fields = line.split()
            out[f"psi_{kind}"] = float(dict(f.split("=") for f in fields)["avg10"])
        for line in (cg / "memory.events").read_text().splitlines():
            k, v = line.split()
            if k in ("high", "oom_kill"):
                out[f"events_{k}"] = int(v)
    except (OSError, ValueError):
        pass
    return out


def pipeline_cgroups() -> dict:
    """{'fence': stats, 'rosbox': stats}. rosbox is the distrobox's own cgroup:
    the console's in-box processes, and any pipeline process ros_bash.sh
    couldn't move (it can't from a cgroup it may not write, e.g. a login
    session scope) — so pipeline memory showing up there means unfenced."""
    out = {}
    fence = fence_cgroup()
    if fence:
        out["fence"] = cgroup_stats(fence)
    for cg in _user_cgroup().glob("user.slice/libpod-*.scope"):
        if "conmon" not in cg.name:
            out["rosbox"] = cgroup_stats(cg)
    return out


def _pss_kb(pid: int) -> int | None:
    try:
        with open(f"/proc/{pid}/smaps_rollup") as f:
            for line in f:
                if line.startswith("Pss:"):
                    return int(line.split()[1])
    except OSError:
        pass
    try:
        with open(f"/proc/{pid}/status") as f:
            for line in f:
                if line.startswith("VmRSS:"):
                    return int(line.split()[1])
    except OSError:
        pass
    return None


def role_of(argv: list[str]) -> str:
    """Script basename for interpreters (`python3 nodes/explorer.py` -> explorer),
    executable basename otherwise (`.../rtabmap_slam/rtabmap` -> rtabmap)."""
    exe = os.path.basename(argv[0])
    if exe.startswith("python"):
        if "-m" in argv[1:]:
            return argv[argv.index("-m") + 1]
        # First non-flag argument is the script: `python3 nodes/x.py`, but also
        # extensionless entry points like `/opt/ros/jazzy/bin/ros2 launch` and
        # `.../rosbridge_server/rosbridge_websocket`.
        script = next((a for a in argv[1:] if not a.startswith("-")), None)
        if script:
            argv = [script] + argv[argv.index(script) + 1:]
            exe = os.path.basename(script)
            if exe.endswith(".py"):
                exe = exe[:-3]
                return "isaac" if exe == "spawn_warehouse" else exe
    if exe == "ros2" and len(argv) > 1:
        return f"ros2_{argv[1]}"
    return exe


def processes() -> list[dict]:
    me = os.getpid()
    out = []
    for d in Path("/proc").iterdir():
        if not d.name.isdigit() or int(d.name) == me:
            continue
        try:
            raw = (d / "cmdline").read_bytes()
        except OSError:
            continue
        if not raw:
            continue
        cmd = raw.decode(errors="replace")
        if not any(m in cmd for m in PIPELINE_MARKS):
            continue
        argv = [a for a in cmd.split("\0") if a]
        # Shell wrappers (`bash -c "...spawn_warehouse.py..."`, setsid, distrobox
        # enter) match on their -c string but hold no memory worth counting, and
        # would steal the role name from the real process.
        if os.path.basename(argv[0]) in ("bash", "sh", "setsid", "distrobox",
                                         "distrobox-enter", "podman", "env",
                                         "systemd-run", "taskset", "conmon"):
            continue
        pss = _pss_kb(int(d.name))
        if pss is None:
            continue
        m = ROBOT_RE.search(cmd.replace("\0", " "))
        out.append({"pid": int(d.name), "robot": f"robot_{m.group(1)}" if m else "shared",
                    "role": role_of(argv), "pss_kb": pss})
    return out


def sample() -> dict:
    mi = meminfo()
    groups: dict[str, int] = {}
    for p in processes():
        key = f"{p['robot']}/{p['role']}"
        groups[key] = groups.get(key, 0) + p["pss_kb"]
    return {
        "wall": round(time.time(), 1),
        "sim_t": sim_time(),
        "mem_total_kb": mi["MemTotal"],
        "mem_avail_kb": mi["MemAvailable"],
        "swap_used_kb": mi["SwapTotal"] - mi["SwapFree"],
        "psi": psi_memory(),
        "gpu_mib": gpu_used_mib(),
        "cgroups": pipeline_cgroups(),
        "groups_kb": groups,
    }


def pipeline_alive() -> bool:
    return any(p["role"] == "isaac" for p in processes())


def cmd_record(args) -> int:
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    stop = {"now": False}
    signal.signal(signal.SIGTERM, lambda *_: stop.update(now=True))
    seen = False
    gone_since = None
    with open(out / "samples.jsonl", "a") as f:
        try:
            while not stop["now"]:
                s = sample()
                alive = any(k.endswith("/isaac") for k in s["groups_kb"])
                if alive:
                    seen, gone_since = True, None
                elif seen:
                    gone_since = gone_since or time.time()
                    # Two intervals of absence, not one: smaps_rollup on a
                    # 10 GB Kit process can transiently fail mid-teardown.
                    if time.time() - gone_since > 2 * args.interval:
                        break
                f.write(json.dumps(s) + "\n")
                f.flush()
                time.sleep(args.interval)
        except KeyboardInterrupt:
            pass
    return 0


def _load(d: Path) -> list[dict]:
    with open(d / "samples.jsonl") as f:
        return [json.loads(l) for l in f if l.strip()]


def _slope(xs: list[float], ys: list[float]) -> float | None:
    if len(xs) < 3:
        return None
    mx, my = sum(xs) / len(xs), sum(ys) / len(ys)
    den = sum((x - mx) ** 2 for x in xs)
    return None if den == 0 else sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / den


def summarize(d: Path, steady_after_s: float) -> str:
    # Only samples with Isaac up: teardown samples (everything 0, sim time
    # frozen at its last value) would drag every fitted slope down.
    rows = [r for r in _load(d) if any(k.endswith("/isaac") for k in r["groups_kb"])]
    # Teardown kills the ROS side a few seconds before Isaac, so the last
    # Isaac-up samples have every rtabmap at 0. Cut from the first sample
    # where the total falls by a fifth: nothing in a live run frees that much
    # at once.
    totals = [sum(r["groups_kb"].values()) for r in rows]
    for i in range(1, len(rows)):
        if totals[i] < 0.8 * totals[i - 1]:
            rows = rows[:i]
            break
    if not rows:
        return f"{d}: no pipeline samples\n"
    timed = [r for r in rows if r["sim_t"] is not None]
    keys = sorted({k for r in rows for k in r["groups_kb"]})
    lines = [f"## {d.name}", ""]
    span = (f"sim t {timed[0]['sim_t']:.0f}→{timed[-1]['sim_t']:.0f} s"
            if timed else "no sim time")
    wall = rows[-1]["wall"] - rows[0]["wall"]
    lines.append(f"{len(rows)} samples over {wall/60:.1f} wall-min, {span}")
    lines.append("")
    lines.append("| group | first MB | last MB | peak MB | MB / sim-min (t>"
                 f"{steady_after_s:.0f}s) |")
    lines.append("|---|---:|---:|---:|---:|")

    def series(key):
        return [(r["sim_t"], r["groups_kb"].get(key, 0) / 1024) for r in timed]

    tot_first = tot_last = tot_peak = 0.0
    for k in keys:
        vals = [r["groups_kb"].get(k, 0) / 1024 for r in rows]
        if max(vals) < 20:  # launch wrappers etc.: noise in a memory table
            continue
        pts = [(t, v) for t, v in series(k) if t >= steady_after_s]
        sl = _slope([t for t, _ in pts], [v for _, v in pts])
        sl_s = f"{sl * 60:+.1f}" if sl is not None else "—"
        lines.append(f"| {k} | {vals[0]:.0f} | {vals[-1]:.0f} | {max(vals):.0f} | {sl_s} |")
    per_sample = [sum(r["groups_kb"].values()) / 1024 for r in rows]
    lines.append(f"| **all pipeline PSS** | {per_sample[0]:.0f} | {per_sample[-1]:.0f} "
                 f"| {max(per_sample):.0f} | |")
    lines.append("")
    avail = [r["mem_avail_kb"] / 1048576 for r in rows]
    gpu = [r["gpu_mib"] for r in rows if r["gpu_mib"] is not None]
    swap = [r["swap_used_kb"] / 1048576 for r in rows]
    psi = [r["psi"].get("some", 0) for r in rows]
    cg = {}
    for r in rows:
        for k, v in r["cgroups"].items():
            if isinstance(v, dict) and "anon" in v:
                cg.setdefault(k, []).append(v["anon"] / 2**30)
    lines.append(f"- MemAvailable min {min(avail):.1f} GB (total "
                 f"{rows[0]['mem_total_kb']/1048576:.1f}); swap peak {max(swap):.2f} GB; "
                 f"PSI some avg10 peak {max(psi):.1f}%")
    if gpu:
        lines.append(f"- GPU memory peak {max(gpu)} MiB (first {gpu[0]})")
    for k, v in cg.items():
        lines.append(f"- cgroup {k} anon: peak {max(v):.2f} GB, last {v[-1]:.2f} GB")
    lines.append("")
    return "\n".join(lines)


def cmd_summarize(args) -> int:
    for d in args.dirs:
        print(summarize(Path(d), args.steady_after))
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    r = sub.add_parser("record")
    r.add_argument("--out", required=True)
    r.add_argument("--interval", type=float, default=10.0)
    r.set_defaults(fn=cmd_record)
    s = sub.add_parser("summarize")
    s.add_argument("dirs", nargs="+")
    s.add_argument("--steady-after", type=float, default=60.0,
                   help="sim seconds to skip before fitting growth (startup allocs)")
    s.set_defaults(fn=cmd_summarize)
    args = ap.parse_args()
    return args.fn(args)


if __name__ == "__main__":
    sys.exit(main())
