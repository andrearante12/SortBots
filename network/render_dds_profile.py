#!/usr/bin/env python3
"""Render per-participant FastDDS XML profiles from network/fastdds_profile.xml.tmpl.

Usage:
    python3 network/render_dds_profile.py --output-dir /tmp/sortbots_dds \
        --ds-host 127.0.0.1 --ds-port 11811 \
        --robot-ids robot_0,robot_1

Writes:
    <output-dir>/profile_root.xml          for Isaac + map_merge + dashboard (root ns)
    <output-dir>/profile_robot_0.xml       for robot_0's ROS 2 stack
    <output-dir>/profile_robot_1.xml       for robot_1's ROS 2 stack

Architecture (Phase 4):
  Root ns: Isaac + DS + map_merge + dashboard.
    profile_root.xml whitelists 127.0.0.1 + 10.77.{i}.1 for each robot i.
    DS is bound 0.0.0.0:11811 (root ns); ROS_DISCOVERY_SERVER=127.0.0.1:11811.

  Robot_i ns: tap mesh IP 10.66.0.{i+1} (inter-robot) + veth IP 10.77.{i}.2 (→DS/Isaac).
    profile_robot_i.xml whitelists these two IPs.
    ROS_DISCOVERY_SERVER=10.77.{i}.1:11811 (host end of veth).

  Note: interfaceWhiteList uses IP addresses, NOT interface names. Interface
  names fail as root inside ip-netns (Phase 3 lesson). DS client config is
  handled by ROS_DISCOVERY_SERVER env var (rmw layer) — NOT in this XML.
  Having both <discoveryProtocol>CLIENT</discoveryProtocol> in XML AND the env
  var causes a conflict where rmw ignores the XML DS list (Phase 3 lesson).
"""
from __future__ import annotations

import argparse
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
TMPL = REPO_ROOT / "network" / "fastdds_profile.xml.tmpl"


def render(tmpl: str, interfaces: list[str]) -> str:
    """Render template with the given IP interface list.

    ds_host / ds_port are no longer embedded in the XML — the rmw layer reads
    ROS_DISCOVERY_SERVER directly. We keep the args for backward compat but
    do not pass them through to the template (template no longer has {{DS_HOST}}
    or {{DS_PORT}} placeholders).
    """
    iface_xml = "\n        ".join(f"<address>{iface}</address>" for iface in interfaces)
    out = tmpl.replace("{{INTERFACES}}", iface_xml)
    return out


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--output-dir", default="/tmp/sortbots_dds")
    # --ds-host / --ds-port kept for backward compatibility (callers may pass them)
    # but are no longer embedded in the rendered XML. DS is configured via
    # ROS_DISCOVERY_SERVER env var set in run_demo.sh per-process.
    parser.add_argument("--ds-host", default="127.0.0.1")
    parser.add_argument("--ds-port", type=int, default=11811)
    parser.add_argument("--robot-ids", default="robot_0,robot_1")
    parser.add_argument("--robots", type=int, default=None)
    args = parser.parse_args()

    out_dir = Path(args.output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    tmpl = TMPL.read_text()
    robot_ids = [r.strip() for r in args.robot_ids.split(",")]
    if args.robots is not None:
        robot_ids = robot_ids[:args.robots]

    # Root profile: Isaac, map_merge, dashboard in root ns.
    # Whitelist: loopback (127.0.0.1) for intra-host topics + the host end of
    # each robot's veth (10.77.{i}.1) so per-robot topics reach Isaac.
    root_ifaces = ["127.0.0.1"] + [f"10.77.{i}.1" for i in range(len(robot_ids))]
    root_profile = render(tmpl, root_ifaces)
    (out_dir / "profile_root.xml").write_text(root_profile)
    print(f"[render_dds_profile] wrote profile_root.xml (ifaces: {', '.join(root_ifaces)})")

    # Per-robot profiles (Phase 4: processes run inside network namespaces).
    # Whitelist: mesh tap IP (10.66.0.{i+1}) for inter-robot DDS over the
    # simulated 802.11s mesh + veth netns IP (10.77.{i}.2) to reach the DS
    # and Isaac in the root ns. Use IPs not interface names (Phase 3 lesson).
    for i, rid in enumerate(robot_ids):
        ifaces = [f"10.66.0.{i + 1}", f"10.77.{i}.2"]
        profile = render(tmpl, ifaces)
        path = out_dir / f"profile_{rid}.xml"
        path.write_text(profile)
        print(f"[render_dds_profile] wrote profile_{rid}.xml (ifaces: {', '.join(ifaces)})")


if __name__ == "__main__":
    main()
