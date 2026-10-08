"""Build / tear down an emulated mesh: one network namespace per node, hwsim
radios, real 802.11s (mesh_fwding 0) + batman-adv, wmediumd SNR matrix.

Run as root INSIDE the commsim VM (never on the host):

    sudo python3 -m commsim.emu.mesh up --n 3 --line      # M0: 0 - 1 - 2, 0 and 2 out of range
    sudo python3 -m commsim.emu.mesh up --n 5 --layout warehouse --seed 1
    sudo python3 -m commsim.emu.mesh down

Deliberately NOT Mininet-WiFi: we need only hwsim + wmediumd + namespaces,
and driving them directly keeps radio/MAC/namespace mapping explicit (plan
§10 fallback). Mininet-WiFi stays installed for its wmediumd build.

Single band (g24) for now; dual band (V4) needs a second radio per node on
its own wmediumd medium — not built yet.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import random
import subprocess
import sys
import time
from pathlib import Path

RUN = Path("/run/commsim")
MESH_ID = "sortbots-mesh"
FREQ = {"g24": 2412, "g5": 5180}


def sh(cmd: str, ns: str | None = None, check=True) -> str:
    if ns:
        cmd = f"ip netns exec {ns} {cmd}"
    p = subprocess.run(cmd, shell=True, text=True, capture_output=True)
    if check and p.returncode:
        raise RuntimeError(f"{cmd!r} -> {p.returncode}: {p.stderr.strip()}")
    return p.stdout


def radio_mac(i: int) -> str:
    return f"02:00:00:00:{i:02x}:00"  # mac80211_hwsim's address for radio i


def phys_by_mac() -> dict:
    out = {}
    for d in Path("/sys/class/ieee80211").iterdir():
        out[(d / "macaddress").read_text().strip()] = d.name
    return out


def snr_matrix(n: int, line: bool, layout: str, seed: int, links: str | None = None) -> tuple[list, list]:
    """Pairwise SNR (dB) and positions. --line: neighbors 25 dB, others none.
    Otherwise the DES propagation model on a layout (same placeholders)."""
    if links:  # explicit topology, e.g. "0-1,0-2,1-3,2-3": 25 dB on listed links only
        pairs = {tuple(sorted(map(int, l.split("-")))) for l in links.split(",")}
        pos = [(5.0 * i, 0.0) for i in range(n)]
        m = [[(25 if tuple(sorted((i, j))) in pairs else -10) for j in range(n)] for i in range(n)]
        return m, pos
    if line:
        pos = [(5.0 * i, 0.0) for i in range(n)]
        m = [[(25 if abs(i - j) == 1 else -10) for j in range(n)] for i in range(n)]
        return m, pos
    from commsim.des import radio as R
    from commsim.des.layout import LAYOUTS
    lay = LAYOUTS[layout]()
    rng = random.Random(seed + 1)
    pos = [lay.gateway_pos] + [lay.random_spot(rng) for _ in range(n - 1)]
    rp = R.RadioParams()
    m = [[0.0] * n for _ in range(n)]
    for i in range(n):
        for j in range(n):
            if i != j:
                racks, shelves = lay.crossings(pos[i], pos[j])
                m[i][j] = R.snr_db(rp, "g24", math.dist(pos[i], pos[j]), racks, shelves,
                                   gateway=0 in (i, j))
    return m, pos


def wmediumd_conf(n: int, m: list, interference: bool = True) -> str:
    # enable_interference: without it wmediumd lets stations transmit over each
    # other with no deferral — the first n=10 runs (2026-10-03) had it OFF.
    ids = ", ".join(f'"{radio_mac(i)}"' for i in range(n))
    links = ",\n    ".join(f"({i}, {j}, {int(round(m[i][j]))})"
                           for i in range(n) for j in range(i + 1, n))
    return f"""ifaces :
{{
  ids = [{ids}];
  enable_interference = {"true" if interference else "false"};
}};
model :
{{
  type = "snr";
  links = (
    {links}
  );
}};
"""


def up(n: int, line: bool, layout: str, seed: int, orig_ms: int, band: str = "g24",
       interference: bool = True, links: str | None = None) -> dict:
    down(quiet=True)
    RUN.mkdir(parents=True, exist_ok=True)
    sh("modprobe batman-adv")
    sh(f"modprobe mac80211_hwsim radios={n}")
    m, pos = snr_matrix(n, line, layout, seed, links)
    (RUN / "wmediumd.cfg").write_text(wmediumd_conf(n, m, interference))
    log = open(RUN / "wmediumd.log", "w")
    wm = subprocess.Popen(["wmediumd", "-l", "5", "-c", str(RUN / "wmediumd.cfg")],
                          stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    time.sleep(1.0)
    if wm.poll() is not None:
        raise RuntimeError(f"wmediumd exited: {(RUN / 'wmediumd.log').read_text()[-800:]}")
    phys = phys_by_mac()
    nodes = []
    for i in range(n):
        ns = f"n{i}"
        phy = phys[radio_mac(i)]
        sh(f"ip netns add {ns}")
        sh(f"iw phy {phy} set netns name {ns}")
        dev = sh("iw dev", ns).split("Interface ")[1].split()[0]
        sh("ip link set lo up", ns)
        sh(f"ip link set {dev} down", ns)
        sh(f"iw dev {dev} set type mp", ns)
        sh(f"ip link set {dev} mtu 1532", ns)  # 1500 + 32 B batman-adv header
        sh(f"ip link set {dev} up", ns)
        sh(f"iw dev {dev} mesh join {MESH_ID} freq {FREQ[band]} HT20", ns)
        # batman-adv does the routing; 802.11s must only provide links.
        sh(f"iw dev {dev} set mesh_param mesh_fwding 0", ns)
        sh(f"iw dev {dev} set mcast_rate 12", ns, check=False)
        # Design: legacy 802.11b rates OFF. Without this, minstrel used 5.5 and
        # 1 Mb/s for ~40% of frames in the first n=10 emulation (2026-10-03).
        sh(f"iw dev {dev} set bitrates legacy-2.4 6 9 12 18 24 36 48 54", ns, check=False)
        sh(f"batctl meshif bat0 interface add {dev}", ns)
        sh(f"batctl meshif bat0 orig_interval {orig_ms}", ns)
        sh("ip link set bat0 up", ns)
        sh(f"ip addr add 10.42.0.{i + 1}/24 dev bat0", ns)
        nodes.append({"ns": ns, "phy": phy, "dev": dev, "ip": f"10.42.0.{i + 1}",
                      "mac": radio_mac(i), "pos": pos[i]})
    state = {"n": n, "interference": interference, "wmediumd_pid": wm.pid, "nodes": nodes, "snr": m, "band": band}
    (RUN / "state.json").write_text(json.dumps(state, indent=2))
    return state


def down(quiet=False) -> None:
    st = RUN / "state.json"
    if st.exists():
        s = json.loads(st.read_text())
        try:
            os.killpg(s["wmediumd_pid"], 15)
        except ProcessLookupError:
            pass
    for ns in sh("ip netns list", check=False).split("\n"):
        ns = ns.split(" ")[0]
        if ns.startswith("n") and ns[1:].isdigit():
            sh(f"ip netns pids {ns} | xargs -r kill", check=False)
            sh(f"ip netns del {ns}", check=False)
    sh("pkill -x wmediumd", check=False)
    time.sleep(0.5)
    sh("modprobe -r mac80211_hwsim", check=False)
    if st.exists():
        st.unlink()
    if not quiet:
        print("mesh down")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="cmd", required=True)
    u = sub.add_parser("up")
    u.add_argument("--n", type=int, required=True)
    u.add_argument("--line", action="store_true")
    u.add_argument("--layout", default="warehouse")
    u.add_argument("--seed", type=int, default=1)
    u.add_argument("--orig-ms", type=int, default=500)
    u.add_argument("--links", default=None, help='explicit topology, e.g. "0-1,0-2,1-3,2-3"')
    sub.add_parser("down")
    a = ap.parse_args(argv)
    if os.geteuid() != 0:
        print("run as root inside the commsim VM", file=sys.stderr)
        return 1
    if a.cmd == "down":
        down()
        return 0
    s = up(a.n, a.line, a.layout, a.seed, a.orig_ms, links=a.links)
    print(json.dumps({k: v for k, v in s.items() if k != "snr"}, indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
