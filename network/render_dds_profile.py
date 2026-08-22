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

All processes run in the root network namespace. Isolation is enforced by
whitelisting each robot's DDS to its own tap device only (for inter-robot
traffic on 10.66.0.x) plus loopback (for per-robot topics with Isaac).
"""
from __future__ import annotations

import argparse
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
TMPL = REPO_ROOT / "network" / "fastdds_profile.xml.tmpl"


def render(tmpl: str, interfaces: list[str], ds_host: str, ds_port: int) -> str:
    iface_xml = "\n        ".join(f"<address>{iface}</address>" for iface in interfaces)
    out = tmpl.replace("{{INTERFACES}}", iface_xml)
    out = out.replace("{{DS_HOST}}", ds_host)
    out = out.replace("{{DS_PORT}}", str(ds_port))
    return out


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--output-dir", default="/tmp/sortbots_dds")
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

    ds_host = args.ds_host
    ds_port = args.ds_port

    # Root profile: Isaac, map_merge, dashboard — loopback only.
    # All per-robot topics (cmd_vel, camera, odom) travel over lo since Isaac
    # and the robot stacks are on the same host. DS is on 127.0.0.1 (lo).
    root_profile = render(tmpl, ["lo"], ds_host, ds_port)
    (out_dir / "profile_root.xml").write_text(root_profile)
    print(f"[render_dds_profile] wrote profile_root.xml (ifaces: lo, DS: {ds_host}:{ds_port})")

    # Per-robot profiles: tap-robot_X (for inter-robot mesh traffic 10.66.0.x)
    # plus loopback (to reach the DS and Isaac's per-robot topics).
    for rid in robot_ids:
        ifaces = [f"tap-{rid}", "lo"]
        profile = render(tmpl, ifaces, ds_host, ds_port)
        path = out_dir / f"profile_{rid}.xml"
        path.write_text(profile)
        print(f"[render_dds_profile] wrote profile_{rid}.xml "
              f"(ifaces: {', '.join(ifaces)}, DS: {ds_host}:{ds_port})")


if __name__ == "__main__":
    main()
