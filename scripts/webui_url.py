#!/usr/bin/env python3
"""Print the tailnet URL to open the SortBots dashboard from a phone.

The dashboard servers (see launch/sortbots_webui.launch.py) all bind 0.0.0.0,
so any device on the Tailscale tailnet can reach them. This resolves the
workstation's MagicDNS name / tailnet IP from `tailscale status --json` and
prints the plain-HTTP URL to open on the phone. If the `qrcode` package is
importable it also renders a terminal QR to scan with the phone camera.

Plain HTTP (not https/Tailscale Serve) on purpose: the dashboard is a 3-port
app (page 8081 + rosbridge ws 9090 + web_video_server 8080) and webui/app.js
derives the rosbridge/video hosts from window.location.hostname, so an HTTPS
origin would make those ws://9090 / http://8080 calls mixed-content and the
browser would block them. An HTTP origin over the private tailnet avoids that.

Run standalone, or let launch/sortbots_webui.launch.py invoke it at startup:

    python3 scripts/webui_url.py --port 8081
"""
from __future__ import annotations

import argparse
import json
import subprocess


def _tailscale_self() -> dict | None:
    """Return the `Self` object from `tailscale status --json`, or None."""
    try:
        out = subprocess.run(
            ["tailscale", "status", "--json"],
            capture_output=True,
            text=True,
            timeout=5,
        )
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return None
    if out.returncode != 0:
        return None
    try:
        return json.loads(out.stdout).get("Self")
    except json.JSONDecodeError:
        return None


def _first_ipv4(ips: list[str]) -> str | None:
    for ip in ips:
        if ":" not in ip:  # skip IPv6
            return ip
    return None


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=8081)
    args = parser.parse_args()

    self_node = _tailscale_self()
    if not self_node:
        print(
            "[webui_url] Tailscale not available (is `tailscale up` running?). "
            "On the same LAN, open http://<this-host-ip>:{}/ instead.".format(args.port)
        )
        return

    dns_name = (self_node.get("DNSName") or "").rstrip(".")
    ipv4 = _first_ipv4(self_node.get("TailscaleIPs") or [])

    host = dns_name or ipv4
    if not host:
        print("[webui_url] Could not resolve a tailnet address for this host.")
        return

    url = f"http://{host}:{args.port}/"
    print("\n" + "=" * 52)
    print("  SortBots dashboard — open on your phone (tailnet):")
    print(f"    {url}")
    if dns_name and ipv4:
        print(f"    (IP fallback: http://{ipv4}:{args.port}/)")
    print("=" * 52 + "\n")

    try:
        import qrcode  # optional; `pip install qrcode` to enable
    except ImportError:
        print("  (pip install qrcode for a scannable QR code here)\n")
        return

    qr = qrcode.QRCode(border=1)
    qr.add_data(url)
    qr.make()
    qr.print_ascii(invert=True)
    print()


if __name__ == "__main__":
    main()
