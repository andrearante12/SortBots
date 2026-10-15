"""Build the fleet's OpenWrt images with the mesh packages already in them.

    python3 -m commsim.hw.firmware --out ~/sortbots-firmware

Asks OpenWrt's own build service (sysupgrade.openwrt.org, "ASU") for a stock
release image plus wpad-mesh and batman-adv, then downloads it and checks the
sha256 the service reports. Baking the packages in replaces README step 2,
which needed internet on every router: GL.iNet routers come up on a fresh
flash with no WAN configured, and the lab's only network is eduroam.

Only the model and package list are sent. Never send the router setup
script (it holds the mesh key): ASU keeps builds in a public store.
Standard library only.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import sys
import time
import urllib.request
from pathlib import Path

ASU = "https://sysupgrade.openwrt.org"
# 25.12.5 = OpenWrt stable on 2026-10-15. Both GL.iNet models are
# mediatek/filogic (MT7981 / MT7986), mt76 driver: 802.11s + SAE supported.
VERSION = "25.12.5"
# The image store answers 403 to urllib's default "Python-urllib/x.y".
HEADERS = {"User-Agent": "sortbots-commsim-firmware/1"}
TARGET = "mediatek/filogic"
# Fleet role -> model. MT6000 (Flint 2): 12 V wall power, 4 LAN ports — the
# gateway, next to the laptop. MT3000 (Beryl AX): 5 V USB-C — runs off a
# robot's power bank.
MODELS = {"gateway": "glinet_gl-mt6000", "robot": "glinet_gl-mt3000"}
PACKAGES = [
    "-wpad-basic-mbedtls",  # no mesh support; conflicts with the full wpad
    "wpad-mesh-mbedtls",    # 802.11s + SAE
    "kmod-batman-adv",
    "batctl-default",
    "luci",                 # web UI for looking at a router by hand; not used by setup
]


def _json(url: str, body: dict | None = None) -> dict:
    req = urllib.request.Request(url, data=json.dumps(body).encode() if body else None,
                                 headers={"Content-Type": "application/json", **HEADERS})
    try:
        with urllib.request.urlopen(req, timeout=60) as r:
            return json.load(r)
    except urllib.error.HTTPError as e:  # ASU answers errors as JSON too
        return json.load(e)


def build(profile: str, version: str = VERSION, timeout_s: int = 900) -> dict:
    d = _json(f"{ASU}/api/v1/build", {"version": version, "target": TARGET,
                                       "profile": profile, "packages": PACKAGES})
    deadline = time.monotonic() + timeout_s
    while d.get("status") == 202:
        if time.monotonic() > deadline:
            raise SystemExit(f"{profile}: build still queued after {timeout_s} s")
        time.sleep(10)
        d = _json(f"{ASU}/api/v1/build/{d['request_hash']}")
    if d.get("status") != 200:
        raise SystemExit(f"{profile}: build failed: {d.get('detail')}\n{d.get('stderr', '')[-2000:]}")
    missing = [p for p in PACKAGES if not p.startswith("-") and p not in d["manifest"]]
    if missing or "wpad-basic-mbedtls" in d["manifest"]:
        raise SystemExit(f"{profile}: image manifest wrong (missing {missing})")
    return d


def fetch(d: dict, out: Path) -> Path:
    img = next(i for i in d["images"] if i["type"] == "sysupgrade")
    dest = out / f"{d['request']['profile'].removeprefix('glinet_')}-openwrt-{d['request']['version']}-mesh-sysupgrade.bin"
    req = urllib.request.Request(f"{ASU}/store/{d['bin_dir']}/{img['name']}", headers=HEADERS)
    with urllib.request.urlopen(req, timeout=120) as r:
        data = r.read()
    if hashlib.sha256(data).hexdigest() != img["sha256"]:
        raise SystemExit(f"{dest.name}: sha256 mismatch")
    dest.write_bytes(data)
    return dest


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--version", default=VERSION)
    a = ap.parse_args(argv)
    a.out.expanduser().mkdir(parents=True, exist_ok=True)
    for role, profile in MODELS.items():
        d = build(profile, a.version)
        path = fetch(d, a.out.expanduser())
        print(f"{role:8s} {path}  sha256 {hashlib.sha256(path.read_bytes()).hexdigest()}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
