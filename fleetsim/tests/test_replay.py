"""fleetsim/replay.py: a recorded run packed for the operator view's replay.
Small floor, a few robots, seconds."""
from __future__ import annotations

import gzip
import json
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from fleetsim import network, replay, sim  # noqa: E402
from fleetsim.tests.test_sim import scenario  # noqa: E402
from simtest import events as ev_lib  # noqa: E402


def record(tmp_path: Path, name: str, scn: dict) -> Path:
    d = tmp_path / name
    d.mkdir()
    (d / "scenario.json").write_text(json.dumps(scn, default=str))
    rec = ev_lib.Recorder(d / "events.jsonl.gz")
    try:
        summary = sim.Sim(scn, recorder=rec).run()
    finally:
        rec.close()
    (d / "summary.json").write_text(json.dumps(summary))
    return d


@pytest.fixture(scope="module")
def runs(tmp_path_factory):
    root = tmp_path_factory.mktemp("fleetsim")
    record(root, "ideal", scenario(
        config={"record": {"belief_maps": ["robot_1"], "belief_map_s": 5.0}},
        disturbances=[{"at_s": 5, "duration_s": 10, "kill": "robot_2"},
                      {"at_s": 3, "blockage": {"rect": [2.0, 2.0, 3.0, 3.0]}}]))
    record(root, "radio", scenario(config={"network": {"model": "radio"}}, duration_s=40,
                                   stop={}))
    return root


def test_a_run_packs_with_its_floor_poses_paths_and_tasks(runs):
    p = replay.pack("ideal", runs)
    g = p["world"]["grid"]
    assert g["w"] * g["h"] > 0 and g["z"]
    assert set(p["world"]["waypoints"]) >= {"shelf_1_0", "dock_0"}
    assert p["robots"] == ["robot_0", "robot_1", "robot_2"]
    assert not p["truncated"]
    # one flat row per 5 Hz sample: t + (x, y, yaw) per robot
    assert len(p["poses"]) > 10 and all(len(r) == 1 + 3 * 3 for r in p["poses"])
    assert [r[0] for r in p["poses"]] == sorted(r[0] for r in p["poses"])
    # every robot that took a task planned a path to it
    assert all(any(len(row[2]) >= 2 for row in p["paths"][rid]) for rid in p["robots"][:2])
    assert {t["state"] for t in p["tasks"].values()} == {"DONE"}
    assert any(m[1] == "done" for m in p["marks"])
    # a terminal task state is followed by the IDLE task_manager goes to
    st = p["states"]["robot_0"]
    k = next(i for i, r in enumerate(st) if r[1] == "DONE")
    assert st[k + 1][1] == "IDLE"


def test_only_the_named_robots_maps_are_snapshotted(runs):
    p = replay.pack("ideal", runs)
    assert list(p["belief_maps"]) == ["robot_1"]
    snap = p["belief_maps"]["robot_1"][0][1]
    assert snap["w"] == p["world"]["grid"]["w"]


def test_disturbances_come_through_as_marks_and_spans(runs):
    p = replay.pack("ideal", runs)
    kinds = [(d.get("on"), next(k for k in d if k in ("kill", "blockage"))) for d in p["disturb"]]
    assert (True, "kill") in kinds and (False, "kill") in kinds and (True, "blockage") in kinds
    assert sum(1 for m in p["marks"] if m[1] == "disturb") == len(p["disturb"])


def test_radio_runs_record_links_and_ideal_runs_do_not(runs):
    assert replay.pack("ideal", runs)["links"] == []
    links = replay.pack("radio", runs)["links"]
    assert links and len(links[0][1]) % 3 == 0 and len(links[0][1]) >= 3


def test_listing_marks_what_can_be_replayed(runs, tmp_path):
    # A run from before the `world` stream: listed, not replayable, a clear error.
    old = runs / "old"
    old.mkdir(exist_ok=True)
    with gzip.open(old / "events.jsonl.gz", "wt") as fh:
        fh.write('{"s":"start","t":0}\n{"s":"gt","t":0,"robots":{},"actors":{}}\n{"s":"end","t":1}\n')
    rows = {r["dir"]: r for r in replay.list_runs(runs)}
    assert rows["ideal"]["replayable"] and rows["ideal"]["finished"] and rows["ideal"]["robots"] == 3
    assert not rows["old"]["replayable"] and not rows["old"]["finished"]
    with pytest.raises(replay.ReplayError, match="re-run"):
        replay.pack("old", runs)


@pytest.mark.parametrize("name", ["../ideal", "a/b", ".hidden", "", "x" * 200])
def test_a_dir_is_a_name_never_a_path(runs, name):
    with pytest.raises(replay.ReplayError):
        replay.pack(name, runs)


def test_a_missing_run_says_so_without_the_path(runs):
    with pytest.raises(FileNotFoundError) as e:
        replay.pack("nope", runs)
    assert str(runs) not in str(e.value)


# ---- the comms monitor (record.net_nodes, 2026-11-26) ----------------------------------

def _net(d: Path) -> list[dict]:
    return [e for e in ev_lib.read(d / "events.jsonl.gz") if e["s"] == "net"]


def test_net_samples_carry_each_nodes_traffic_and_it_adds_up(runs):
    for name in ("ideal", "radio"):
        for ev in _net(runs / name):
            nk = ev["node_kinds"]
            assert set(nk) <= set(ev["kinds"])
            for kind, st in ev["kinds"].items():
                rows = nk.get(kind, {})
                tot = {f: sum(rows.get(f, [0])) for f in ("sent", "bytes", "lost", "rx")}
                # per node, sender side and receiver side, sums to the fleet's counters
                assert (tot["sent"], tot["bytes"], tot["lost"], tot["rx"]) == \
                    (st["sent"], st["bytes"], st["lost"], st["delivered"]), (name, ev["t"], kind)
                assert all(len(v) == 4 for v in rows.values())  # gateway + 3 robots
                assert st["msgs"] <= st["sent"]
            if name == "radio":
                assert len(ev["node_load"]) == len(ev["node_air"]) == 4
                assert ev["load_max"] == pytest.approx(max(ev["node_load"]), abs=1e-3)
                assert all("air_ms" in st for st in ev["kinds"].values())
            else:  # an ideal network has no airtime, so no load to record
                assert "node_load" not in ev and "node_air" not in ev
                assert all("air_ms" not in st for st in ev["kinds"].values())


def test_recording_per_node_traffic_changes_nothing_else(tmp_path):
    """record.net_nodes is recording only: the run with it off is the run with
    it on, minus the new fields (sim.py, network.py take_nodes)."""
    def events(on: bool, name: str):
        d = record(tmp_path, name, scenario(config={"network": {"model": "radio"},
                                                   "record": {"net_nodes": on}},
                                           duration_s=30, stop={}))
        out = []
        for e in ev_lib.read(d / "events.jsonl.gz"):
            e = {k: v for k, v in e.items() if k not in ("wall", "wall_s", "summary", "node_kinds",
                                                         "node_load", "node_air", "ogm_air")}
            out.append(e)
        return out
    on, off = events(True, "on"), events(False, "off")
    assert len(on) == len(off) and on == off
    assert any(e["s"] == "net" for e in off)


def test_the_monitor_stream_packs_per_sample_and_per_node(runs):
    for name, n_nodes in (("ideal", 4), ("radio", 4)):
        c = replay.pack(name, runs)["comms"]
        assert c["kinds"][0] == "status" and "tasks" in c["kinds"]
        assert c["cap"] == network.LOAD_CAP
        width = 6 + len(c["fields"]) * len(c["kinds"])
        assert c["rows"] and all(len(r) == width for r in c["rows"])
        assert len(c["nodes"]) == len(c["rows"])
        assert [r[0] for r in c["rows"]] == [r[0] for r in c["nodes"]]
        # dt is the sample's own window, never a sliver (the end-of-run sample)
        assert all(r[1] >= 0.25 for r in c["rows"])
        st = c["kinds"].index("status")
        o = 6 + len(c["fields"]) * st
        t, _load, _air, per = c["nodes"][5]
        sent = per[st][c["node_fields"].index("sent")]
        assert sum(sent) == c["rows"][5][o] and len(sent) == n_nodes and t == c["rows"][5][0]
        if name == "radio":
            assert c["model"] == "radio" and c["contention"] == "global"
            assert all(len(r[1]) == n_nodes for r in c["nodes"])
            assert c["rows"][5][o + c["fields"].index("air_ms")] > 0
        else:
            assert c["contention"] is None and all(r[1] is None for r in c["nodes"])
            assert c["rows"][5][o + c["fields"].index("air_ms")] is None


def test_a_run_recorded_before_the_monitor_still_replays(runs, tmp_path):
    """Strip what 2026-11-26 added: the run packs, the monitor gets the fleet
    rows it always had and null where it has nothing."""
    old = runs / "pre_monitor"
    old.mkdir(exist_ok=True)
    for f in ("scenario.json", "summary.json"):
        (old / f).write_text((runs / "radio" / f).read_text())
    with gzip.open(old / "events.jsonl.gz", "wt") as fh:
        for e in ev_lib.read(runs / "radio" / "events.jsonl.gz"):
            if e["s"] == "net":
                e = {k: v for k, v in e.items() if not k.startswith("node_") and k != "ogm_air"}
                e["kinds"] = {k: {f: v for f, v in st.items() if f not in ("msgs", "air_ms")}
                              for k, st in e["kinds"].items()}
            fh.write(json.dumps(e) + "\n")
    p = replay.pack("pre_monitor", runs)
    c = p["comms"]
    assert p["poses"] and c["rows"] and all(r[3] is None for r in c["nodes"])
    assert all(r[1] is None and r[2] is None for r in c["nodes"])
    msgs = 6 + c["fields"].index("msgs")
    assert all(r[msgs] is None for r in c["rows"])
    assert c["rows"][5][3] is not None  # load_max was always recorded


# ---- bench runs: bench/<bench>/<run>, confined to RESULTS_DIR ----------------------------

@pytest.fixture(scope="module")
def results(runs, tmp_path_factory):
    """A results dir as fleetsim bench leaves it: one plain run, one bench of two
    runs with its summary.json, and the traps a public route must not walk into."""
    import shutil
    root = tmp_path_factory.mktemp("results")
    shutil.copytree(runs / "ideal", root / "plain")
    b = root / "bench" / "20261126T010203Z_smoke"
    for run_name, src in (("delivery_smoke_r3_s1_radio", "radio"), ("delivery_smoke_r3_s1_default", "ideal")):
        shutil.copytree(runs / src, b / run_name)
    (b / "summary.json").write_text(json.dumps({
        "started": "2026-11-26T01:02:03Z", "complete": True, "counts": {"pass": 2},
        "suite": {"name": "smoke", "description": "two runs"},
        "runs": [{"name": "delivery_smoke_r3_s1_default", "variant": "default", "status": "pass"},
                 {"name": "delivery_smoke_r3_s1_radio", "variant": "radio", "status": "pass"}]}))
    (b / "results.csv").write_text("run\n")
    outside = tmp_path_factory.mktemp("outside")
    shutil.copytree(runs / "ideal", outside / "secret_run")
    (b / "linked_out").symlink_to(outside / "secret_run")          # a run dir leading out
    (root / "linked_in").symlink_to(b / "delivery_smoke_r3_s1_radio")  # one staying inside
    ob = root / "bench" / "20261126T020000Z_escape"
    ob.mkdir()
    (ob / "run").symlink_to(outside / "secret_run")
    (outside / "x").mkdir()
    (root / "bench" / "20261126T030000Z_linked").symlink_to(outside / "x")  # a bench dir leading out
    return root


def test_benches_are_listed_with_their_runs_in_plan_order(results):
    assert sorted(r["dir"] for r in replay.list_runs(results)) == ["linked_in", "plain"]
    bs = replay.list_benches(results)
    assert [b["dir"] for b in bs] == ["20261126T010203Z_smoke"]  # the escapes have no runs
    b = bs[0]
    assert (b["suite"], b["runs"], b["complete"], b["counts"]) == ("smoke", 2, True, {"pass": 2})
    rows = replay.list_bench("20261126T010203Z_smoke", results)
    assert [r["dir"] for r in rows] == ["bench/20261126T010203Z_smoke/delivery_smoke_r3_s1_default",
                                        "bench/20261126T010203Z_smoke/delivery_smoke_r3_s1_radio"]
    assert [r["variant"] for r in rows] == ["default", "radio"]
    assert rows[1]["network"] == "radio" and rows[1]["replayable"] and rows[1]["finished"]


def test_a_bench_run_replays_by_its_reference(results):
    ref = "bench/20261126T010203Z_smoke/delivery_smoke_r3_s1_radio"
    p = replay.pack(ref, results)
    assert p["dir"] == ref and p["network"] == "radio" and p["links"] and p["comms"]["rows"]
    assert replay.pack("linked_in", results)["network"] == "radio"  # a link that stays inside


@pytest.mark.parametrize("ref", [
    "bench", "bench/20261126T010203Z_smoke", "bench/../plain", "bench/./x/y",
    "bench/20261126T010203Z_smoke/../plain", "bench//delivery_smoke_r3_s1_radio",
    "other/20261126T010203Z_smoke/delivery_smoke_r3_s1_radio",
    "bench/20261126T010203Z_smoke/delivery_smoke_r3_s1_radio/x", "/etc/passwd", "bench/.x/y",
    "bench\\20261126T010203Z_smoke\\delivery_smoke_r3_s1_radio", None, 7])
def test_a_reference_is_names_never_a_path(results, ref):
    assert not replay.valid_ref(ref)
    with pytest.raises(replay.ReplayError):
        replay.pack(ref, results)


@pytest.mark.parametrize("ref", ["bench/20261126T010203Z_smoke/linked_out",
                                 "bench/20261126T020000Z_escape/run",
                                 "bench/20261126T030000Z_linked/anything",
                                 "bench/20261126T010203Z_smoke/results.csv"])
def test_nothing_resolving_outside_the_results_dir_is_served(results, ref):
    assert replay.valid_ref(ref)
    with pytest.raises(FileNotFoundError) as e:
        replay.pack(ref, results)
    assert str(results) not in str(e.value)


def test_a_bench_listing_takes_a_name_and_stays_inside(results):
    for bad in ("../x", "a/b", ".hidden", "", None):
        with pytest.raises(replay.ReplayError):
            replay.list_bench(bad, results)
    for missing in ("nope", "20261126T030000Z_linked"):
        with pytest.raises(FileNotFoundError):
            replay.list_bench(missing, results)
    assert replay.list_bench("20261126T020000Z_escape", results) == []


def test_side_files_linked_out_are_not_read(results, tmp_path):
    d = results / "bench" / "20261126T010203Z_smoke" / "delivery_smoke_r3_s1_default"
    secret = tmp_path / "secret.json"
    secret.write_text(json.dumps({"ok": True, "metrics": [{"id": "leak"}]}))
    (d / "score.json").unlink(missing_ok=True)
    (d / "score.json").symlink_to(secret)
    replay._cache.clear()
    p = replay.pack("bench/20261126T010203Z_smoke/delivery_smoke_r3_s1_default", results)
    assert p["score"] is None
