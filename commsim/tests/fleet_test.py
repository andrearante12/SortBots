import json

import pytest

from commsim.core import fleet as F

BASE = F.Path(__file__).resolve().parent.parent / "configs" / "base.yaml"


def cfg(**fl):
    c = F.load_config(BASE)
    c["fleet"].update(fl)
    return c


def test_ids_addresses_and_macs():
    nodes = F.build_fleet(cfg(robots=3))
    assert [n.name for n in nodes] == ["gateway", "robot_1", "robot_2", "robot_3"]
    assert [n.ip for n in nodes] == ["10.42.0.1", "10.42.0.2", "10.42.0.3", "10.42.0.4"]
    assert nodes[2].macs == {"g24": "02:01:00:00:00:02", "g5": "02:02:00:00:00:02"}


def test_robot_id_must_fit_uint8():
    with pytest.raises(ValueError):
        F.build_fleet(cfg(robots=300, subnet="10.42.0.0/16"))


def test_generated_configs_pass_isolation_check(tmp_path):
    c = cfg(robots=4)
    F.generate(c, tmp_path)
    assert F.check_dir(tmp_path, c["fleet"]["subnet"]) == []
    eps = {n: json.loads((tmp_path / n / "zenohd.json5").read_text())["connect"]["endpoints"]
           for n in ("gateway", "robot_1", "robot_4")}
    # lower id dials: gateway dials nobody, robot_4 dials 0..3 -> one session per pair
    assert eps["gateway"] == [] and eps["robot_1"] == ["tcp/10.42.0.1:7447"]
    assert len(eps["robot_4"]) == 4 and "tcp/10.42.0.5:7447" not in eps["robot_4"]
    z = json.loads((tmp_path / "robot_1" / "zenohd.json5").read_text())
    assert z["transport"]["shared_memory"]["enabled"] is False


@pytest.mark.parametrize("mutate, needle", [
    (lambda z: z["listen"]["endpoints"].append("tcp/0.0.0.0:7447"), "every interface"),
    (lambda z: z["connect"]["endpoints"].append("tcp/192.168.1.50:7447"), "outside fleet subnet"),
    (lambda z: z["connect"]["endpoints"].append("tcp/127.0.0.1:7447"), "outside fleet subnet"),
    (lambda z: z["scouting"]["multicast"].update(enabled=True), "multicast"),
    (lambda z: z["transport"]["shared_memory"].update(enabled=True), "shared_memory"),
])
def test_isolation_check_flags_violations(mutate, needle):
    c = cfg(robots=2)
    nodes = F.build_fleet(c)
    z = F.zenoh_router_config(nodes[1], nodes, c)
    mutate(z)
    bad = F.check_zenoh_config(z, c["fleet"]["subnet"])
    assert any(needle in v for v in bad), bad


def test_spec_connect_all_dials_every_peer(tmp_path):
    c = F.load_config(BASE, BASE.parent / "spec_original.yaml")
    c["fleet"]["robots"] = 3
    F.generate(c, tmp_path)
    z = json.loads((tmp_path / "gateway" / "zenohd.json5").read_text())
    assert len(z["connect"]["endpoints"]) == 3


def test_timer_order_base_ok_spec_flagged():
    from commsim.core.timers import check
    assert check(F.load_config(BASE)) == []
    bad = check(F.load_config(BASE, BASE.parent / "spec_original.yaml"))
    assert any("worst Status gap" in b for b in bad)
    # the design's 500 ms OGM: with TCP Status 5.67 s + backoff > 6 s timeout;
    # with UDP Status the measured 5.67 s gap just clears it
    c500 = F.load_config(BASE, BASE.parent / "ogm500.yaml")
    c500["protocol"]["status_transport"] = "tcp"
    assert any("worst Status gap" in v for v in check(c500))
    # worst reroute at 500 ms (5.67 s) exceeds the 4 s zenoh lease whatever the
    # transport — the lease check used to compare against the 1.95 s best case
    c500["protocol"]["status_transport"] = "udp"
    assert any("worst measured reroute" in v for v in check(c500))
    assert check(F.load_config(BASE, BASE.parent / "ogm250.yaml")) == []


def test_fleet_gen_refuses_timer_violations(tmp_path):
    spec = [str(BASE), str(BASE.parent / "spec_original.yaml")]
    assert F.main(["gen", *spec, "--out", str(tmp_path / "a")]) == 2
    assert not (tmp_path / "a").exists()
    assert F.main(["gen", *spec, "--out", str(tmp_path / "b"), "--allow-timer-violations"]) == 0


def test_sweep_scripts_pin_what_they_compare():
    import re
    root = BASE.parents[1] / "scripts"
    fix = (root / "fix_sweep.sh").read_text()
    for line in fix.splitlines():
        if re.search(r"(tcp|udp|\$T)-500", line):
            assert "$O500" in line and "--status-transport" in line, line
        if re.search(r"(tcp|udp|\$T)-250", line):
            assert "$O250" in line and "--status-transport" in line, line
    final = (root / "final_sweep.sh").read_text()
    assert 'RUN="' in final and "spec_original.yaml" in final.split('RUN="', 1)[1].split('"', 1)[0]


def test_status_udp_bound_to_mesh_ip_and_checked(tmp_path):
    c = cfg(robots=2)
    F.generate(c, tmp_path)
    u = json.loads((tmp_path / "robot_1" / "status_udp.json").read_text())
    assert u["bind"] == "10.42.0.2:7450"
    assert sorted(u["peers"]) == ["10.42.0.1:7450", "10.42.0.3:7450"]
    assert F.check_dir(tmp_path, c["fleet"]["subnet"]) == []
    assert any("every interface" in v for v in
               F.check_status_udp_config({"bind": "0.0.0.0:7450", "peers": []}, "10.42.0.0/24"))
