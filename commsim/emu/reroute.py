"""Measure batman-adv reroute time on a diamond: 0 -(1|2)- 3.

Ping 0 -> 3 every 10 ms, block the first hop of the active path on both
ends (iw ... station set <MAC> plink_action block — the spec's injection),
and report the outage = longest run of lost pings. Unblock, wait, repeat.

    sudo python3 -m commsim.emu.reroute --trials 6 --orig-ms 500
"""
import argparse
import json
import re
import subprocess
import sys
import time

from commsim.emu import mesh


def sh(cmd, ns=None, check=True):
    return mesh.sh(cmd, ns, check)


def nexthop(ns, dst_mac):
    for l in sh("batctl meshif bat0 o", ns).splitlines():
        if dst_mac in l and l.strip().startswith("*"):
            return l.split()[-3] if "[" in l else None
    return None


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--trials", type=int, default=6)
    ap.add_argument("--orig-ms", type=int, default=500)
    ap.add_argument("--out", default="/run/commsim/reroute.json")
    a = ap.parse_args(argv)
    st = mesh.up(4, False, "warehouse", 1, a.orig_ms, links="0-1,0-2,1-3,2-3")
    devs = {n["ns"]: n["dev"] for n in st["nodes"]}
    macs = {n["ns"]: n["mac"] for n in st["nodes"]}
    time.sleep(20)
    sh("ping -c 3 -W 1 10.42.0.4", "n0", check=False)
    results = []
    for t in range(a.trials):
        time.sleep(8)
        o = sh("batctl meshif bat0 o", "n0")
        line = [l for l in o.splitlines() if macs["n3"] in l and l.strip().startswith("*")]
        hop_mac = re.findall(r"02:00:00:00:0[12]:00", line[0])[0] if line else macs["n1"]
        relay = "n1" if hop_mac == macs["n1"] else "n2"
        ping = subprocess.Popen(f"ip netns exec n0 ping -D -i 0.01 -W 1 -w 12 10.42.0.4",
                                shell=True, text=True, stdout=subprocess.PIPE)
        time.sleep(3)
        t_block = time.time()
        sh(f"iw dev {devs['n0']} station set {macs[relay]} plink_action block", "n0")
        sh(f"iw dev {devs[relay]} station set {macs['n0']} plink_action block", relay)
        out, _ = ping.communicate()
        seqs = sorted(int(m) for m in re.findall(r"icmp_seq=(\d+)", out))
        gaps = [b - a_ for a_, b in zip(seqs, seqs[1:])]
        outage_s = (max(gaps) - 1) * 0.01 if gaps else None
        after = nexthop("n0", macs["n3"])
        results.append({"trial": t, "blocked": f"n0-{relay}", "outage_s": outage_s,
                        "received": len(seqs)})
        print(json.dumps(results[-1]), flush=True)
        sh(f"iw dev {devs['n0']} station set {macs[relay]} plink_action open", "n0", check=False)
        sh(f"iw dev {devs[relay]} station set {macs['n0']} plink_action open", relay, check=False)
    mesh.down(quiet=True)
    outs = sorted(r["outage_s"] for r in results if r["outage_s"] is not None)
    summary = {"orig_interval_ms": a.orig_ms, "trials": results,
               "outage_median_s": outs[len(outs) // 2] if outs else None,
               "outage_max_s": outs[-1] if outs else None}
    open(a.out, "w").write(json.dumps(summary, indent=1))
    print(json.dumps({k: v for k, v in summary.items() if k != "trials"}))
    return 0


if __name__ == "__main__":
    sys.exit(main())
