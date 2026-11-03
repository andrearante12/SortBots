"""fleetsim/replay.py: a recorded run packed for the operator view's replay.
Small floor, a few robots, seconds."""
from __future__ import annotations

import gzip
import json
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from fleetsim import replay, sim  # noqa: E402
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
