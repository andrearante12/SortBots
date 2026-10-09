import csv
import os
import shutil
import subprocess

import pytest
import yaml

from commsim.core import fleet as F
from commsim.hw import deploy as D
from commsim.hw import probe as P

CONFIGS = F.Path(__file__).resolve().parent.parent / "configs"
KEY = "test-mesh-key-0123456789"


def lab_cfg():
    return F.load_config(CONFIGS / "base.yaml", CONFIGS / "hw_lab.yaml")


def bundle(tmp_path, cfg=None):
    out = tmp_path / "hw"
    out.mkdir()
    D.deploy(cfg or lab_cfg(), out, KEY, "test")
    return out


def test_lab_bundle_layout_and_addresses(tmp_path):
    out = bundle(tmp_path)
    assert sorted(p.name for p in out.iterdir() if p.is_dir()) == ["gateway", "robot_1", "robot_2"]
    s = (out / "robot_1" / "router-setup.sh").read_text()
    assert "uci set network.lan.ipaddr=10.42.0.253" in s
    assert "addresses: [10.42.0.2/24]" in (out / "robot_1" / "netplan-sortbots-mesh.yaml").read_text()
    # the hardware runs the same zenoh/udp configs the emulation measured, and they stay isolated
    assert F.check_dir(out, "10.42.0.0/24") == []


def test_router_script_carries_the_measured_design(tmp_path):
    s = (bundle(tmp_path) / "gateway" / "router-setup.sh").read_text()
    for needle in ("mesh_fwding=0", "encryption=sae", "orig_interval=250", "routing_algo=BATMAN_IV",
                   "mtu=1532", "mcast_rate=12000", "dhcp.lan.ignore=1", "network.wan.auto=0",
                   "legacy-2.4 6 9 12"):
        assert needle in s, needle
    # 2.4 GHz-only lab config -> no 5 GHz mesh interface
    assert "mesh_g24" in s and "mesh_g5" not in s


def test_dual_band_adds_5ghz(tmp_path):
    c = lab_cfg()
    c["fleet"]["bands"] = ["g24", "g5"]
    s = (bundle(tmp_path, c) / "robot_2" / "router-setup.sh").read_text()
    assert "radio_for 5g" in s and "uci set wireless.$r.channel=36" in s


@pytest.mark.skipif(not shutil.which("dash"), reason="needs dash for a POSIX sh parse")
def test_router_script_is_valid_posix_sh(tmp_path):
    p = bundle(tmp_path) / "robot_1" / "router-setup.sh"
    assert subprocess.run(["dash", "-n", str(p)]).returncode == 0


def test_key_only_in_router_scripts_and_private(tmp_path):
    out = bundle(tmp_path)
    for f in out.rglob("*"):
        if f.is_file():
            assert (KEY in f.read_text()) == (f.name == "router-setup.sh"), f
            if f.name == "router-setup.sh":
                assert f.stat().st_mode & 0o077 == 0
    assert out.stat().st_mode & 0o077 == 0


def test_host_has_no_route_or_dns_onto_the_mesh(tmp_path):
    n = yaml.safe_load((bundle(tmp_path) / "robot_1" / "netplan-sortbots-mesh.yaml").read_text())
    eth = n["network"]["ethernets"]["enP8p1s0"]
    assert eth == {"dhcp4": False, "dhcp6": False, "accept-ra": False, "addresses": ["10.42.0.2/24"]}


def test_nodes_only_talk_to_their_local_router(tmp_path):
    env = (bundle(tmp_path) / "robot_1" / "node.env").read_text()
    assert 'connect/endpoints=["tcp/127.0.0.1:7447"]' in env
    assert "scouting/multicast/enabled=false" in env and "10.42.0." not in env


def test_router_and_host_addresses_never_collide():
    c = lab_cfg()
    c["fleet"]["subnet"] = "10.42.0.0/28"   # 14 usable addresses
    c["fleet"]["robots"] = 6                # hosts .1-.7, routers .14-.8: fits
    D.check_addresses(F.build_fleet(c), c)
    c["fleet"]["robots"] = 7
    with pytest.raises(ValueError):
        D.check_addresses(F.build_fleet(c), c)


def test_key_file_rules(tmp_path):
    k = tmp_path / "k"
    key = D.read_key(k, new=True)
    assert len(key) >= 8 and k.stat().st_mode & 0o777 == 0o600
    with pytest.raises(SystemExit):
        D.read_key(k, new=True)              # never overwrite an existing key
    os.chmod(k, 0o644)
    with pytest.raises(SystemExit):
        D.read_key(k, new=False)             # world-readable key refused
    os.chmod(k, 0o600)
    k.write_text("short\n")
    with pytest.raises(SystemExit):
        D.read_key(k, new=False)


def test_bundle_refused_inside_tracked_repo_paths():
    with pytest.raises(SystemExit):
        D._must_be_private(D.REPO / "commsim" / "hw_bundle")
    D._must_be_private(D.REPO / "commsim" / "results" / "hw")  # gitignored: allowed


def test_spec_original_timers_refused(tmp_path):
    rc = D.main([str(CONFIGS / "base.yaml"), str(CONFIGS / "spec_original.yaml"),
                 "--out", str(tmp_path / "o"), "--mesh-key-file", str(tmp_path / "k"), "--new-key"])
    assert rc == 2 and not (tmp_path / "o").exists()


def test_probe_summary_finds_the_outage(tmp_path):
    p = tmp_path / "p.csv"
    with open(p, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["seq", "send_ns", "recv_ns"])
        for i in range(400):  # 100 Hz; probes sent 1.00-2.50 s are lost
            t = i * 10_000_000
            lost = 100 <= i < 250
            w.writerow([i, t, "" if lost else t + 2_000_000])
    s = P.summary(str(p))
    assert "delivered 0.6250" in s and "rtt p50 2.0 ms" in s and "longest outage 1.51 s" in s
