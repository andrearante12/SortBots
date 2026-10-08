"""M1: real zenohd + rmw_zenoh + Status stubs over the emulated mesh.

Every node (gateway n0 + robots) runs its own rmw_zenohd (fleet-generated
override: explicit peers, mesh IP only, 2 s lease) and a Status stub
publishing at --hz to one shared topic (V2-like all-to-all status). Captures
TCP on the gateway's bat0 to measure what one Status costs on the wire.

    sudo python3 -m commsim.emu.m1 --n 3 --line --duration 120 --out /run/commsim/m1
"""
from __future__ import annotations

import argparse
import csv
import glob
import json
import os
import re
import statistics
import subprocess
import sys
import time
from pathlib import Path

from commsim.core import fleet as F
from commsim.emu import mesh

REPO = Path(__file__).resolve().parents[2]
ROS = "source /opt/ros/jazzy/setup.bash && source {ws}/install/setup.bash"


def nsrun(ns: str, cmd: str, env: dict, log: Path) -> subprocess.Popen:
    exports = " ".join(f"{k}='{v}'" for k, v in env.items())
    full = (f"ip netns exec {ns} env -i HOME=/root PATH=/usr/bin:/bin {exports} "
            f"bash -c \"{ROS.format(ws=REPO / 'commsim/ws')} && exec {cmd}\"")
    return subprocess.Popen(full, shell=True, stdout=open(log, "w"), stderr=subprocess.STDOUT,
                            start_new_session=True)


def pct(xs, q):
    s = sorted(xs)
    return s[min(len(s) - 1, max(0, int(round(q / 100 * len(s))) - 1))] if s else float("nan")


def analyze(out: Path, n: int, hz: float, duration: float, warmup: float) -> dict:
    rows = []
    for f in glob.glob(str(out / "stub_*.csv")):
        rows += list(csv.DictReader(open(f)))
    t0 = min(int(r["send_ns"]) for r in rows) if rows else 0
    steady = [r for r in rows if int(r["send_ns"]) - t0 > warmup * 1e9]
    lat = [(int(r["recv_ns"]) - int(r["send_ns"])) / 1e6 for r in steady]
    # Delivery against what was SENT (sent_*.csv). The first version built the
    # denominator from received rows, so a Status nobody got vanished from it
    # and delivery was overstated (code review 2026-10-04). Sends in the last
    # second are excluded: they may still be in flight when the stubs stop.
    sent_rows = [r for f in glob.glob(str(out / "sent_*.csv")) for r in csv.DictReader(open(f))]
    if sent_rows:
        t0s = min(int(r["send_ns"]) for r in sent_rows)
        t_end = max(int(r["send_ns"]) for r in sent_rows) - 1e9
        sent = {(r["src"], int(r["seq"])) for r in sent_rows
                if int(r["send_ns"]) - t0s > warmup * 1e9 and int(r["send_ns"]) < t_end}
        got = {(r["src"], int(r["seq"]), r["dst"]) for r in rows
               if (r["src"], int(r["seq"])) in sent}
        delivery = len(got) / (len(sent) * (n - 1)) if sent else None
    else:  # older runs without a send log: the old, overstated figure
        sent = {}
        for r in steady:
            sent.setdefault(r["src"], set()).add(int(r["seq"]))
        expected = sum(len(v) for v in sent.values()) * (n - 1)
        delivery = len(steady) / expected if expected else None
    res = {"n": n, "hz": hz, "samples": len(lat),
           "p50_ms": pct(lat, 50), "p95_ms": pct(lat, 95), "p99_ms": pct(lat, 99),
           "max_ms": max(lat) if lat else None,
           "delivery": delivery, "delivery_from_send_log": bool(sent_rows),
           "delivery_lower_bound": delivery}  # old key kept for existing readers
    # wire cost from the gateway's capture: TCP payload lengths and pure ACKs
    txt = subprocess.run(f"tcpdump -nn -r {out}/gw_bat0.pcap 'tcp port 7447 or udp port 7450'",
                         shell=True, text=True, capture_output=True).stdout
    lens = [int(m) for m in re.findall(r"length (\d+)", txt)]
    data = [x for x in lens if x > 0]
    res.update({
        "cap_tcp_segments": len(lens), "cap_pure_acks": sum(1 for x in lens if x == 0),
        "cap_data_segments": len(data),
        "cap_data_len_median": statistics.median(data) if data else None,
        "cap_small_segments_le16B": sum(1 for x in data if x <= 16),
        "cap_seconds": duration,
    })
    return res


def airtime(pcap: Path, duration: float) -> dict:
    """Busy fraction from the hwsim0 capture, using the DES's 802.11 timing.
    hwsim0 shows every transmission incl. ACK frames (wmediumd-modeled), and
    hwsim's rate control picks LEGACY OFDM rates (12/24/48 Mb/s), not HT —
    a real difference from the DES, which assumes HT MCS rates."""
    from commsim.des import radio as R
    txt = subprocess.run(f"tcpdump -nn -e -r {pcap}", shell=True, text=True,
                         capture_output=True).stdout
    busy, frames, acks, rates = 0.0, 0, 0, {}
    for line in txt.splitlines():
        if not line[:1].isdigit():
            continue
        if "Acknowledgment" in line:
            busy += (R.SIFS_US["g24"] + R._ofdm_us(R.ACK_BYTES, 24.0, R.LEGACY_PREAMBLE_US, "g24")) * 1e-6
            acks += 1
            continue
        m_len = re.search(r"length (\d+)", line)
        if not m_len:
            continue
        size = int(m_len.group(1)) + 30  # tcpdump length excludes the 802.11 header/FCS
        mcs = re.search(r"MCS (\d+)", line)
        leg = re.search(r"([\d.]+) Mb/s", line)
        if mcs:
            i = int(mcs.group(1))
            rate = R.HT_RATES[i % 8][0] * (2 if i >= 8 else 1)
            t = R.frame_airtime_us(size, rate, "g24", unicast=False)
        else:
            rate = float(leg.group(1)) if leg and float(leg.group(1)) > 0 else 6.0
            t = R.frame_airtime_us(size, rate, "g24", unicast=False, legacy=True)
        rates[rate] = rates.get(rate, 0) + 1
        busy += t * 1e-6
        frames += 1
    return {"air_frames": frames, "air_acks": acks, "air_rates": rates,
            "air_busy_frac": busy / duration if duration else None}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=3)
    ap.add_argument("--line", action="store_true")
    ap.add_argument("--layout", default="warehouse")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--hz", type=float, default=3.0)
    ap.add_argument("--duration", type=float, default=120.0)
    ap.add_argument("--warmup", type=float, default=20.0)
    ap.add_argument("--out", type=Path, default=Path("/run/commsim/m1"))
    ap.add_argument("--depth", type=int, default=None, help="default: protocol.status_qos_depth")
    ap.add_argument("--transport", choices=["zenoh", "udp"], default=None,
                    help="default: protocol.status_transport from the config")
    ap.add_argument("--configs", nargs="*", default=[], help="extra YAML layers, e.g. spec_original.yaml")
    ap.add_argument("--links", default=None, help='explicit topology, e.g. "0-1,0-2,1-3,2-3"')
    ap.add_argument("--orig-ms", type=int, default=500)
    ap.add_argument("--lease-ms", type=int, default=None, help="override zenoh transport lease")
    ap.add_argument("--block-at", type=float, default=None,
                    help="seconds into the stub run: block n0's link to its active next hop toward the last node")
    a = ap.parse_args(argv)
    if os.geteuid() != 0:
        print("run as root inside the commsim VM", file=sys.stderr)
        return 1
    a.out.mkdir(parents=True, exist_ok=True)
    for f in a.out.glob("*"):
        f.unlink()
    state = mesh.up(a.n, a.line, a.layout, a.seed, a.orig_ms, links=a.links)
    (a.out / "mesh_state.json").write_text(json.dumps(state))
    cfg = F.load_config(REPO / "commsim/configs/base.yaml", *a.configs)
    if a.transport is None:
        a.transport = "udp" if cfg["protocol"].get("status_transport") == "udp" else "zenoh"
    if a.depth is None:
        a.depth = int(cfg["protocol"].get("status_qos_depth", 32))
    cfg["fleet"]["robots"] = a.n - 1
    if a.lease_ms:
        cfg["zenoh"]["transport_lease_ms"] = a.lease_ms
    fleet = F.build_fleet(cfg)
    assert [nd.ip for nd in fleet] == [s["ip"] for s in state["nodes"]], "fleet/mesh IP mismatch"
    print("waiting for batman to converge...", flush=True)
    time.sleep(15)
    reach = subprocess.run(
        " && ".join(f"ip netns exec n0 ping -c 2 -W 1 -q {nd.ip} >/dev/null" for nd in fleet[1:]),
        shell=True).returncode == 0
    print(f"gateway reaches every robot: {reach}", flush=True)

    procs = []
    for i, nd in enumerate(fleet):
        ov = F.zenoh_override(F.zenoh_router_config(nd, fleet, cfg))
        procs.append(nsrun(f"n{i}", "ros2 run rmw_zenoh_cpp rmw_zenohd",
                           {"RMW_IMPLEMENTATION": "rmw_zenoh_cpp", "ZENOH_CONFIG_OVERRIDE": ov},
                           a.out / f"zenohd_{i}.log"))
    time.sleep(5)
    cap = subprocess.Popen(f"ip netns exec n0 tcpdump -nn -i bat0 -w {a.out}/gw_bat0.pcap",
                           shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                           start_new_session=True)
    # hwsim0 sees every frame any hwsim radio transmits (radiotap: rate, length)
    # -> channel busy time computed with the DES's own 802.11 timing.
    subprocess.run("ip link set hwsim0 up", shell=True)
    air = subprocess.Popen(f"tcpdump -nn -e -i hwsim0 -s 64 -w {a.out}/hwsim0.pcap",
                           shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                           start_new_session=True)
    node_env = {"RMW_IMPLEMENTATION": "rmw_zenoh_cpp",
                # nodes talk only to their own router; never let SHM cross namespaces
                "ZENOH_CONFIG_OVERRIDE": "transport/shared_memory/enabled=false"}
    stubs = [nsrun(f"n{i}",
                   f"python3 {REPO}/commsim/stubs/status_stub.py --robot-id {nd.robot_id} "
                   f"--hz {a.hz} --duration {a.duration} --depth {a.depth} --out {a.out}/stub_{i}.csv "
                   f"--sent-out {a.out}/sent_{i}.csv "
                   f"--transport {a.transport} --bind {nd.ip} "
                   f"--udp-port {cfg['protocol'].get('status_udp_port', 7450)} "
                   f"--peers {','.join(x.ip for x in fleet if x != nd)}",
                   node_env, a.out / f"stub_{i}.log")
             for i, nd in enumerate(fleet)]
    if a.block_at is not None:
        time.sleep(a.block_at)
        last = state["nodes"][-1]
        o = mesh.sh("batctl meshif bat0 o", "n0")
        line = [l for l in o.splitlines() if last["mac"] in l and l.strip().startswith("*")][0]
        hop = re.findall(r"02:00:00:00:[0-9a-f]{2}:00", line)[1]
        relay = next(nd for nd in state["nodes"] if nd["mac"] == hop)
        mesh.sh(f"iw dev {state['nodes'][0]['dev']} station set {hop} plink_action block", "n0")
        mesh.sh(f"iw dev {relay['dev']} station set {state['nodes'][0]['mac']} plink_action block",
                relay["ns"])
        (a.out / "block.json").write_text(json.dumps({"t_ns": time.time_ns(), "relay": relay["ns"]}))
        print(f"blocked n0-{relay['ns']}", flush=True)
        time.sleep(max(0, a.duration / 2 - a.block_at))
    else:
        time.sleep(a.duration / 2)
    # CPU/RAM per emulated robot, sampled mid-run while everything is up
    mem = subprocess.run("ps -C rmw_zenohd,python3 -o rss=,pcpu=,comm=", shell=True, text=True,
                         capture_output=True).stdout
    for s in stubs:
        s.wait()
    os.killpg(cap.pid, 2)
    os.killpg(air.pid, 2)
    time.sleep(1)
    res = analyze(a.out, a.n, a.hz, a.duration, a.warmup)
    res.update(airtime(a.out / "hwsim0.pcap", a.duration))
    res["gateway_reaches_all"] = reach
    res["rss_mb_per_node"] = round(sum(int(l.split()[0]) for l in mem.split("\n")
                                       if l.strip()) / 1024 / a.n, 1)
    res["cpu_pct_per_node"] = round(sum(float(l.split()[1]) for l in mem.split("\n")
                                        if l.strip()) / a.n, 1)
    res["depth"] = a.depth
    res["transport"] = a.transport
    res["zenoh_connect"] = cfg["zenoh"].get("connect")
    res["zenoh_lease_ms"] = cfg["zenoh"]["transport_lease_ms"]
    res["rss_mb_total"] = round(sum(int(l.split()[0]) for l in mem.split("\n") if l.strip()) / 1024, 1)
    for p in procs:
        try:
            os.killpg(p.pid, 15)
        except ProcessLookupError:
            pass
    mesh.down(quiet=True)
    (a.out / "result.json").write_text(json.dumps(res, indent=2))
    print(json.dumps(res, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
