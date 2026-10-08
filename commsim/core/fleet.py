"""One fleet list -> every node's addresses, radio MACs and middleware config.

Pure python (PyYAML only), no ROS: it runs on the host for offline tests and
inside the VM to generate what each emulated namespace loads.

    python3 -m commsim.core.fleet gen commsim/configs/base.yaml --out OUT_DIR
    python3 -m commsim.core.fleet check OUT_DIR --subnet 10.42.0.0/24

`check` is the static half of scripts/check_isolation.sh: a config that
listens or connects outside the fleet subnet (e.g. on the eduroam interface
of an operator laptop) or leaves multicast scouting on is a violation.
"""
from __future__ import annotations

import argparse
import copy
import ipaddress
import json
import re
import sys
from dataclasses import dataclass, field
from pathlib import Path

import yaml

BAND_INDEX = {"g24": 1, "g5": 2}


def deep_merge(base: dict, over: dict) -> dict:
    out = copy.deepcopy(base)
    for k, v in (over or {}).items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = deep_merge(out[k], v)
        else:
            out[k] = copy.deepcopy(v)
    return out


def load_config(*paths: str | Path) -> dict:
    """Merge YAML layers left to right (base, variant, layout, scenario, sweep)."""
    cfg: dict = {}
    for p in paths:
        with open(p) as f:
            raw = yaml.safe_load(f) or {}
        if not isinstance(raw, dict):
            raise ValueError(f"{p}: expected a mapping")
        cfg = deep_merge(cfg, raw)
    return cfg


@dataclass(frozen=True)
class Node:
    robot_id: int
    name: str
    ip: str
    is_gateway: bool
    macs: dict = field(default_factory=dict, compare=False, hash=False)


def build_fleet(cfg: dict) -> list[Node]:
    fl = cfg["fleet"]
    net = ipaddress.ip_network(fl["subnet"])
    gw = int(fl.get("gateway_id", 0))
    n = int(fl["robots"])
    ids = [gw] + [i for i in range(1, n + 2) if i != gw][:n]
    # robot_id is uint8 on the wire (FleetHeader); the host part must also fit.
    if max(ids) > 255:
        raise ValueError(f"robot_id {max(ids)} does not fit uint8")
    if max(ids) + 1 >= net.num_addresses - 1:
        raise ValueError(f"subnet {net} too small for {n} robots + gateway")
    bands = fl.get("bands", ["g24"])
    nodes = []
    for rid in ids:
        macs = {b: f"02:{BAND_INDEX[b]:02x}:00:00:00:{rid:02x}" for b in bands}
        nodes.append(Node(
            robot_id=rid,
            name="gateway" if rid == gw else f"robot_{rid}",
            ip=str(net.network_address + 1 + rid),
            is_gateway=rid == gw,
            macs=macs,
        ))
    return nodes


def zenoh_router_config(node: Node, fleet: list[Node], cfg: dict) -> dict:
    """zenohd config for one machine. Nodes reach it on tcp/localhost:7447."""
    port = int(cfg["fleet"].get("zenoh_port", 7447))
    z = cfg.get("zenoh", {})
    return {
        "mode": "router",
        "listen": {"endpoints": [f"tcp/{node.ip}:{port}", f"tcp/127.0.0.1:{port}"]},
        # lower_id: dial only lower ids -> one TCP session per pair (see base.yaml)
        "connect": {"endpoints": [f"tcp/{p.ip}:{port}" for p in fleet if p != node and (
            z.get("connect", "lower_id") == "all" or p.robot_id < node.robot_id)]},
        # Explicit peers from the fleet list only: no multicast discovery, no
        # gossip — discovery must never depend on whatever else is on the air.
        "scouting": {"multicast": {"enabled": False}, "gossip": {"enabled": False}},
        "transport": {
            "link": {"tx": {
                "lease": int(z.get("transport_lease_ms", 10000)),
                "keep_alive": int(z.get("keep_alive", 4)),
            }},
            # Emulated robots share the host's /dev/shm; SHM would let data
            # skip the radio entirely and make every latency number a lie.
            "shared_memory": {"enabled": bool(z.get("shared_memory", False))},
        },
    }


def zenoh_override(conf: dict) -> str:
    """The same router config as a ZENOH_CONFIG_OVERRIDE string, layered on
    rmw_zenoh's DEFAULT_RMW_ZENOH_ROUTER_CONFIG. Those defaults are a 60 s
    lease / keep_alive 2 — far outside our timer ordering, so the override is
    load-bearing, not cosmetic."""
    def j(v):
        return json.dumps(v).replace(" ", "")
    tx = conf["transport"]["link"]["tx"]
    return ";".join([
        f"listen/endpoints={j(conf['listen']['endpoints'])}",
        f"connect/endpoints={j(conf['connect']['endpoints'])}",
        "scouting/multicast/enabled=false",
        "scouting/gossip/enabled=false",
        f"transport/link/tx/lease={tx['lease']}",
        f"transport/link/tx/keep_alive={tx['keep_alive']}",
        f"transport/shared_memory/enabled={j(conf['transport']['shared_memory']['enabled'])}",
    ])


def status_udp_config(node: Node, fleet: list[Node], cfg: dict) -> dict:
    """UDP Status endpoint for one machine. Bound to the MESH address only —
    never 0.0.0.0, which would also accept packets on an operator laptop's
    campus Wi-Fi interface. Recipients: every peer (V2-style). Neighbor-only
    recipient selection over UDP needs a subscription mechanism (open item)."""
    port = int(cfg["protocol"].get("status_udp_port", 7450))
    return {"bind": f"{node.ip}:{port}",
            "peers": [f"{p.ip}:{port}" for p in fleet if p != node]}


def cyclone_config_xml(node: Node, fleet: list[Node]) -> str:
    peers = "\n".join(f'        <Peer Address="{p.ip}"/>' for p in fleet if p != node)
    return f"""<?xml version="1.0" encoding="UTF-8"?>
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain Id="any">
    <General>
      <Interfaces><NetworkInterface address="{node.ip}"/></Interfaces>
      <AllowMulticast>false</AllowMulticast>
    </General>
    <Discovery>
      <ParticipantIndex>auto</ParticipantIndex>
      <Peers>
{peers}
      </Peers>
    </Discovery>
  </Domain>
</CycloneDDS>
"""


def generate(cfg: dict, out: Path) -> list[Node]:
    fleet = build_fleet(cfg)
    out.mkdir(parents=True, exist_ok=True)
    for node in fleet:
        d = out / node.name
        d.mkdir(exist_ok=True)
        (d / "zenohd.json5").write_text(
            json.dumps(zenoh_router_config(node, fleet, cfg), indent=2) + "\n")
        (d / "cyclonedds.xml").write_text(cyclone_config_xml(node, fleet))
        (d / "status_udp.json").write_text(
            json.dumps(status_udp_config(node, fleet, cfg), indent=2) + "\n")
        (d / "zenoh_override.txt").write_text(
            zenoh_override(zenoh_router_config(node, fleet, cfg)) + "\n")
    (out / "fleet.json").write_text(json.dumps(
        [{"robot_id": n.robot_id, "name": n.name, "ip": n.ip,
          "is_gateway": n.is_gateway, "macs": n.macs} for n in fleet], indent=2) + "\n")
    return fleet


_ENDPOINT = re.compile(r"^(tcp|udp|quic|tls)/\[?([0-9a-fA-F:.]+)\]?:(\d+)$")


def check_zenoh_config(conf: dict, subnet: str) -> list[str]:
    """Violations of the network-isolation rules for one zenohd config."""
    net = ipaddress.ip_network(subnet)
    bad = []
    for section in ("listen", "connect"):
        for ep in conf.get(section, {}).get("endpoints", []):
            m = _ENDPOINT.match(ep)
            if not m:
                bad.append(f"{section}: unparseable or non-IP endpoint {ep!r}")
                continue
            addr = ipaddress.ip_address(m.group(2))
            if addr.is_unspecified:
                bad.append(f"{section}: {ep} binds every interface (incl. campus Wi-Fi)")
            elif not (addr in net or (section == "listen" and addr.is_loopback)):
                bad.append(f"{section}: {ep} is outside fleet subnet {net}")
    scout = conf.get("scouting", {})
    if scout.get("multicast", {}).get("enabled", True):
        bad.append("scouting.multicast is not disabled")
    if conf.get("transport", {}).get("shared_memory", {}).get("enabled", False):
        bad.append("transport.shared_memory is enabled")
    return bad


def check_status_udp_config(conf: dict, subnet: str) -> list[str]:
    net = ipaddress.ip_network(subnet)
    bad = []
    for label, ep in [("bind", conf["bind"])] + [("peer", e) for e in conf["peers"]]:
        addr = ipaddress.ip_address(ep.rsplit(":", 1)[0])
        if addr.is_unspecified:
            bad.append(f"status udp {label} {ep} binds every interface (incl. campus Wi-Fi)")
        elif addr not in net:
            bad.append(f"status udp {label} {ep} is outside fleet subnet {net}")
    return bad


def check_dir(out: Path, subnet: str) -> list[str]:
    bad = []
    files = sorted(out.glob("*/zenohd.json5"))
    if not files:
        return [f"{out}: no */zenohd.json5 found"]
    for f in files:
        bad += [f"{f}: {v}" for v in check_zenoh_config(json.loads(f.read_text()), subnet)]
    for f in sorted(out.glob("*/status_udp.json")):
        bad += [f"{f}: {v}" for v in check_status_udp_config(json.loads(f.read_text()), subnet)]
    return bad


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    g = sub.add_parser("gen")
    g.add_argument("configs", nargs="+")
    g.add_argument("--out", required=True, type=Path)
    g.add_argument("--allow-timer-violations", action="store_true",
                   help="write configs anyway (e.g. to reproduce spec_original.yaml)")
    c = sub.add_parser("check")
    c.add_argument("dir", type=Path)
    c.add_argument("--subnet", required=True)
    a = ap.parse_args(argv)
    if a.cmd == "gen":
        cfg = load_config(*a.configs)
        from commsim.core.timers import check as check_timers
        bad = check_timers(cfg)
        for v in bad:
            print(f"TIMER ORDER: {v}", file=sys.stderr)
        if bad and not a.allow_timer_violations:
            print("refusing to write configs (--allow-timer-violations to override)", file=sys.stderr)
            return 2
        fleet = generate(cfg, a.out)
        print(f"generated {len(fleet)} nodes in {a.out}")
        return 0
    bad = check_dir(a.dir, a.subnet)
    for v in bad:
        print(f"VIOLATION {v}", file=sys.stderr)
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
