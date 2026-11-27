"""Tests for webui/fleetsim_editor.py and its serve.py routes (the scenario
editor in the operator view's Tests tab). No ROS, no sim; the floor preview
runs fleetsim/preview.py as the console does, a /usr/bin/python3 child.

    /usr/bin/python3 -m pytest tests/fleetsim_editor_test.py
"""
from __future__ import annotations

import json
import shutil
import sys
import threading
import urllib.error
import urllib.request
from http.server import ThreadingHTTPServer
from pathlib import Path

import pytest
import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT))
sys.path.insert(0, str(REPO_ROOT / "webui"))

import fleetsim_editor as fe  # noqa: E402
import serve  # noqa: E402
from fleetsim import config  # noqa: E402

SCENARIOS = sorted(p.stem for p in (REPO_ROOT / "fleetsim" / "scenarios").glob("*.yaml"))


def browser(v):
    """What a value looks like after JSON through the page: 1.0 comes back as 1."""
    v = json.loads(json.dumps(v))
    if isinstance(v, float) and v.is_integer():
        return int(v)
    if isinstance(v, dict):
        return {k: browser(x) for k, x in v.items()}
    if isinstance(v, list):
        return [browser(x) for x in v]
    return v


@pytest.fixture
def scn_dir(tmp_path, monkeypatch):
    """A copy of fleetsim/scenarios/ that saves go to."""
    d = tmp_path / "scenarios"
    shutil.copytree(REPO_ROOT / "fleetsim" / "scenarios", d)
    monkeypatch.setattr(config, "SCENARIOS_DIR", d)
    fe._cache.clear()
    return d


@pytest.fixture
def maps(tmp_path, monkeypatch):
    """A maps library with one 8 x 6 m map, walls round the edge."""
    d = tmp_path / "maps" / "tiny"
    d.mkdir(parents=True)
    w, h = 160, 120
    px = bytearray([254]) * (w * h)
    for x in range(w):
        px[x] = px[(h - 1) * w + x] = 0
    for y in range(h):
        px[y * w] = px[y * w + w - 1] = 0
    (d / "grid.pgm").write_bytes(f"P5\n{w} {h}\n255\n".encode() + bytes(px))
    (d / "grid.yaml").write_text("image: grid.pgm\nresolution: 0.05\norigin: [-1.0, -2.0, 0.0]\n"
                                 "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n")
    (d / "waypoints.json").write_text(json.dumps({"waypoints": [
        {"name": "a", "x": 1.0, "y": 0.0}, {"name": "b", "x": 5.0, "y": 2.0}]}))
    monkeypatch.setenv("SORTBOTS_MAPS_DIR", str(tmp_path / "maps"))
    fe._cache.clear()
    return tmp_path / "maps"


def minimal(**over):
    raw = {"name": "edit_t", "robots": 3, "duration_s": 120,
           "tasks": [{"at_s": 1, "pickup": "shelf_1_0", "dropoff": "dock_0"}]}
    raw.update(over)
    return raw


def save(raw, **kw):
    return fe.save({"scenario": raw, "header": kw.pop("header", ""), **kw})


# ---------------------------------------------------------------------------
# reading
# ---------------------------------------------------------------------------

def test_catalog_lists_every_scenario_with_its_title_and_description():
    c = fe.catalog()
    assert [s["name"] for s in c["scenarios"]] == SCENARIOS
    smoke = next(s for s in c["scenarios"] if s["name"] == "delivery_smoke")
    assert smoke["title"].startswith("Eight robots") and "Twelve fixed" in smoke["description"]
    assert set(c["disturbance_kinds"]) == config.DISTURBANCE_KINDS
    assert "world" in c["base"] and any(m["name"] == "fleet_tasks" for m in c["metrics"])


def test_read_gives_the_mapping_the_header_and_comments_by_section():
    r = fe.read("map_sharing")
    assert r["scenario"]["name"] == "map_sharing"
    assert r["header"].startswith("Does sharing map patches")
    assert "#" not in r["header"].splitlines()[0]
    assert {"section": "tasks", "text": "a minute apart: this asks about the map, "
                                        "not about queueing at one shelf"} in r["comments"]
    assert any(c["section"] == "metrics" for c in r["comments"])


@pytest.mark.parametrize("bad", ["../etc/passwd", "a/b", "Walkers", ""])
def test_read_refuses_names_that_are_not_scenario_names(bad):
    with pytest.raises(fe.EditorError) as e:
        fe.read(bad)
    assert e.value.code == 400


def test_read_of_a_missing_scenario_names_it_not_its_path():
    with pytest.raises(fe.EditorError) as e:
        fe.read("no_such_scenario")
    assert e.value.code == 404 and "/" not in str(e.value)


# ---------------------------------------------------------------------------
# round trip
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("name", SCENARIOS)
def test_every_scenario_round_trips_unchanged(scn_dir, name):
    before = config.load(name)
    original = (scn_dir / f"{name}.yaml").read_text()
    r = fe.read(name)
    out = save(browser(r["scenario"]), header=r["header"], overwrite=True, base_sha=r["sha"])
    assert out["ok"] and out["dropped"] == []
    # Nothing was edited, so not a byte changes: header, comments, 1.0 and all.
    assert (scn_dir / f"{name}.yaml").read_text() == original
    assert config.load(name) == before


@pytest.mark.parametrize("name", SCENARIOS)
def test_every_scenario_written_fresh_resolves_the_same(scn_dir, name):
    # "New from": the writer's own layout, no old file to copy sections from.
    r = fe.read(name)
    text, _ = fe.dump(fe.keep_types(browser(r["scenario"]), r["scenario"]), r["header"])
    assert text.startswith("# " + r["header"].splitlines()[0])
    assert config.resolve(yaml.safe_load(text)) == config.load(name)


def test_an_edit_keeps_the_header_and_every_section_it_did_not_touch(scn_dir):
    r = fe.read("map_sharing")
    original = (scn_dir / "map_sharing.yaml").read_text()
    raw = browser(r["scenario"])
    raw["tasks"].append({"at_s": 400, "pickup": "shelf_1_3", "dropoff": "dock_8"})
    out = save(raw, header=r["header"], overwrite=True, base_sha=r["sha"])
    text = (scn_dir / "map_sharing.yaml").read_text()
    head = original.split("name:")[0]
    assert text.startswith(head)  # the rationale header, as it was
    assert "    # Every robot ends up with the fleet's newest view" in text  # metrics untouched
    assert "pass: {done_frac: {gte: 1.0}}" in text  # 1.0, not 1
    assert "tasks:  # a minute apart" not in text
    assert [c["section"] for c in out["dropped"]] == ["tasks"]
    assert len(config.load("map_sharing")["tasks"]) == 7


def test_new_from_carries_the_sources_sections_and_writes_a_new_file(scn_dir):
    r = fe.read("walkers")
    raw = browser(r["scenario"])
    raw["name"] = "walkers_fast"
    raw["actors"][0]["path"]["speed_mps"] = 1.6
    out = save(raw, header=r["header"], source="walkers")
    assert out["created"] and out["file"] == "fleetsim/scenarios/walkers_fast.yaml"
    text = (scn_dir / "walkers_fast.yaml").read_text()
    assert "# Issue #18's case" in text and "    # First cut (seeds 1-4" in text
    assert config.load("walkers_fast")["actors"][0]["path"]["speed_mps"] == 1.6
    assert (scn_dir / "walkers.yaml").read_text() == (REPO_ROOT / "fleetsim/scenarios/walkers.yaml").read_text()


def test_the_header_is_written_as_comments(scn_dir):
    save(minimal(), header="Why this exists.\n\n  indented: kept")
    assert (scn_dir / "edit_t.yaml").read_text().startswith("# Why this exists.\n#\n#   indented: kept\nname: edit_t\n")


# ---------------------------------------------------------------------------
# what may be written, and where
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("name", ["../evil", "a/b", "..", "Evil", "", "x" * 65, 7, None, "-x"])
def test_save_refuses_names_that_could_leave_the_scenarios_dir(scn_dir, name):
    before = sorted(p.name for p in scn_dir.iterdir())
    with pytest.raises(fe.EditorError) as e:
        save(minimal(name=name))
    assert e.value.code == 400 and e.value.where == "name"
    assert sorted(p.name for p in scn_dir.iterdir()) == before


def test_save_writes_only_the_one_file(scn_dir):
    before = {p.name for p in scn_dir.iterdir()}
    save(minimal())
    assert {p.name for p in scn_dir.iterdir()} - before == {"edit_t.yaml"}


def test_save_never_writes_through_a_symlink(scn_dir, tmp_path):
    outside = tmp_path / "outside.yaml"
    outside.write_text("keep me\n")
    (scn_dir / "evil.yaml").symlink_to(outside)
    with pytest.raises(fe.EditorError):
        save(minimal(name="evil"), overwrite=True)
    assert outside.read_text() == "keep me\n"


@pytest.mark.parametrize("value", ["/etc", "../maps/tiny", "tiny/../../x", "~/maps/tiny", "not_in_library",
                                   7, ["tiny"], {"$yaml": "/etc"}])
def test_world_map_takes_only_a_library_name(scn_dir, maps, value):
    raw = minimal(config={"world": {"source": "map", "map": value}}, tasks=[])
    res = fe.check(raw)
    assert not res["ok"] and res["where"] == "config.world.map", res
    with pytest.raises(fe.EditorError):
        save(raw)


def test_a_library_map_is_drawn_and_saved(scn_dir, maps):
    raw = minimal(config={"world": {"source": "map", "map": "tiny", "cell_m": 0.1}},
                  tasks=[{"at_s": 1, "pickup": "a", "dropoff": "b"}])
    res = fe.check(raw, grid=True)
    assert res["ok"], res
    w = res["world"]
    assert w["origin"] == [-1.0, -2.0] and w["size_m"] == [8.0, 6.0] and w["grid"]["res"] == 0.1
    assert set(w["waypoints"]) == {"a", "b"} and len(w["homes"]) == 3
    assert {m["name"] for m in fe.catalog()["maps"]} == {"tiny"}
    save(raw)
    assert config.load("edit_t")["config"]["world"]["map"] == "tiny"


@pytest.mark.parametrize("over,where", [
    ({"config": {"world": {"actors": "../configs/worlds/x"}}}, "config.world.actors"),
    ({"config": {"fleet_tasks": {"log": "/tmp/x.log"}}}, "config.fleet_tasks.log"),
    ({"config": {"network": {"radio": {"radio_model": {"file": "..\\x"}}}}}, "config.network.radio.radio_model.file"),
])
def test_no_path_gets_in_anywhere_under_config(scn_dir, over, where):
    res = fe.check(minimal(**over))
    assert not res["ok"] and res["where"] == where, res


# ---------------------------------------------------------------------------
# checking
# ---------------------------------------------------------------------------

def test_yaml_typed_values_read_like_set(scn_dir):
    raw = minimal(config={"map_share": {"enabled": {"$yaml": "false"}},
                          "record": {"belief_maps": {"$yaml": "[robot_0, robot_2]"}}})
    assert fe.check(raw)["ok"]
    save(raw)
    cfg = config.load("edit_t")["config"]
    assert cfg["map_share"]["enabled"] is False and cfg["record"]["belief_maps"] == ["robot_0", "robot_2"]


@pytest.mark.parametrize("text,why", [("[1,", "isn't a YAML value"), ("2026-11-26", "date"),
                                      (".nan", "finite"), ("!!binary aGk=", "bytes")])
def test_yaml_that_is_not_plain_data_is_refused_where_it_was_typed(scn_dir, text, why):
    res = fe.check(minimal(config={"status": {"hz": {"$yaml": text}}}))
    assert not res["ok"] and res["where"] == "config.status.hz" and why in res["error"], res


def test_json_nan_is_refused():
    res = fe.check(minimal(duration_s=float("nan")))
    assert not res["ok"] and res["where"] == "duration_s"


def test_resolve_errors_come_back_with_where_they_are(scn_dir):
    res = fe.check(minimal(config={"status": {"hzz": 3}}))
    assert not res["ok"] and res["where"] == "config.status.hzz" and "no such setting" in res["error"]
    res = fe.check(minimal(tasks=[{"at_s": 1, "pickup": "shelf_1_0"}]))
    assert res["where"] == "tasks[0]"
    res = fe.check(minimal(metrics=[{"metric": "fleet_tasks", "pass": {"nope": {"gte": 1}}}]))
    assert res["where"] == "metrics[0].pass.nope", res


@pytest.mark.parametrize("over,where", [
    ({"disturbances": [{"at_s": 5, "kill": "robot_3"}]}, "disturbances[0].kill"),
    ({"disturbances": [{"at_s": 5, "duration_s": 9, "partition": [["robot_0", "robot_7"], "rest"]}]},
     "disturbances[0].partition"),
])
def test_a_robot_the_fleet_lacks_is_a_resolve_error(scn_dir, over, where):
    # config.resolve refuses these since the scenario drafts (2026-11-26): at
    # 2 robots a fixed robot_7 used to crash the run mid-way.
    res = fe.check(minimal(**over))
    assert not res["ok"] and res["where"] == where, res
    with pytest.raises(fe.EditorError) as e:
        save(minimal(**over))
    assert e.value.code == 400 and e.value.where == where


def test_scaling_disturbances_pass_the_floor_check(scn_dir):
    res = fe.check(minimal(disturbances=[
        {"at_s": 5, "kill": {"select": "carrying", "within_s": 60}},
        {"at_s": 9, "duration_s": 9, "partition": [{"first": 0.5}, "rest"]},
        {"at_s": 30, "duration_s": 9, "partition": [["gateway", {"last": 0.3}], "rest"]}]))
    assert res["ok"], res


@pytest.mark.parametrize("over,where", [
    ({"tasks": [{"at_s": 1, "pickup": "shelf_99_0", "dropoff": "dock_0"}]}, "tasks[0].pickup"),
    ({"task_arrival": {"rate_hz": 0.1, "pickups": "rack_*", "dropoffs": "dock_*"}}, "task_arrival.pickups"),
    ({"disturbances": [{"at_s": 5, "blockage": {"rect": [58, 1, 70, 3]}}]}, "disturbances[0].blockage"),
    # more robots than the apron and the free floor have room for
    ({"robots": 900, "config": {"world": {"synthetic": {"width_m": 30, "height_m": 22}}}}, "robots"),
])
def test_floor_problems_block_the_save(scn_dir, over, where):
    res = fe.check(minimal(**over))
    assert not res["ok"] and where in [p["where"] for p in res["problems"] if p["level"] == "error"], res
    with pytest.raises(fe.EditorError) as e:
        save(minimal(**over))
    assert e.value.code == 400 and e.value.where == where


def test_a_warning_does_not_block_the_save(scn_dir):
    raw = minimal(disturbances=[{"at_s": 500, "kill": "robot_1"}])  # after the 120 s run ends
    res = fe.check(raw)
    assert res["ok"] and res["problems"][0]["level"] == "warning"
    assert save(raw)["ok"]


def test_synthetic_options_change_the_floor(scn_dir):
    res = fe.check(minimal(config={"world": {"synthetic": {"width_m": 30, "height_m": 22, "docks": 4}}}), grid=True)
    w = res["world"]
    assert w["size_m"] == [30.0, 22.0] and sorted(k for k in w["waypoints"] if k.startswith("dock_")) == \
        ["dock_0", "dock_1", "dock_2", "dock_3"]
    assert w["synthetic_defaults"]["aisle_m"] > 0  # what the page offers as placeholders


@pytest.mark.parametrize("syn", [{"width_m": 5000}, {"width_m": 0}, {"width_m": "60"},
                                 {"width_m": 300, "height_m": 300}])
def test_a_floor_too_big_for_the_console_is_refused_before_it_is_built(scn_dir, syn):
    cfg = {"world": {"synthetic": syn, "cell_m": 0.05 if "height_m" in syn else 0.2}}
    res = fe.check(minimal(config=cfg))
    assert not res["ok"] and res["where"].startswith("config.world"), res


def test_an_existing_file_needs_overwrite_and_the_sha_it_was_opened_at(scn_dir):
    r = fe.read("delivery_smoke")
    raw = browser(r["scenario"])
    with pytest.raises(fe.EditorError) as e:
        save(raw, header=r["header"])
    assert e.value.code == 409 and "already exists" in str(e.value)
    with pytest.raises(fe.EditorError) as e:
        save(raw, header=r["header"], overwrite=True, base_sha="0" * 64)
    assert e.value.code == 409 and "changed on disk" in str(e.value)
    assert save(raw, header=r["header"], overwrite=True, base_sha=r["sha"])["ok"]


def test_where_of_takes_the_most_specific_path():
    assert fe.where_of("actors: actors[0].path.speed_mps: must be > 0") == "actors[0].path.speed_mps"
    assert fe.where_of("config.world.cell_m: expected 0.05..0.5 m") == "config.world.cell_m"
    assert fe.where_of("a scenario must be a YAML mapping") is None


# ---------------------------------------------------------------------------
# the routes: save is gated exactly like launching a run
# ---------------------------------------------------------------------------

@pytest.fixture
def server(scn_dir, monkeypatch):
    srv = ThreadingHTTPServer(("127.0.0.1", 0), serve.DashboardHandler)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    monkeypatch.setattr(serve, "FLEET", None)

    def call(path, body=None):
        req = urllib.request.Request(f"http://127.0.0.1:{srv.server_port}{path}",
                                     data=None if body is None else json.dumps(body).encode(),
                                     headers={"Content-Type": "application/json"})
        try:
            with urllib.request.urlopen(req, timeout=30) as r:
                return r.status, json.loads(r.read())
        except urllib.error.HTTPError as e:
            return e.code, json.loads(e.read())

    yield call
    srv.shutdown()
    srv.server_close()


def test_save_needs_the_sim_console_like_launching(server, scn_dir, monkeypatch):
    code, body = server("/api/fleetsim/editor/save", {"scenario": minimal()})
    assert code == 503 and "sim dashboard console" in body["error"]
    assert not (scn_dir / "edit_t.yaml").exists()
    monkeypatch.setattr(serve, "FLEET", object())  # what --control on the sim platform sets
    code, body = server("/api/fleetsim/editor/save", {"scenario": minimal()})
    assert code == 200 and body["ok"] and (scn_dir / "edit_t.yaml").exists()
    code, body = server("/api/fleetsim/editor/save", {"scenario": minimal()})
    assert code == 409 and body["where"] == "name"


def test_reading_and_checking_need_no_control(server):
    code, body = server("/api/fleetsim/editor")
    assert code == 200 and body["control"] is False and [s["name"] for s in body["scenarios"]] == SCENARIOS
    code, body = server("/api/fleetsim/editor/scenario?name=walkers")
    assert code == 200 and body["scenario"]["name"] == "walkers"
    code, body = server("/api/fleetsim/editor/scenario?name=../walkers")
    assert code == 400
    code, body = server("/api/fleetsim/editor/check", {"scenario": minimal(), "grid": True})
    assert code == 200 and body["ok"] and body["world"]["grid"]["w"] == 300
    code, body = server("/api/fleetsim/editor/check", {"scenario": minimal(robots=0)})
    assert code == 200 and not body["ok"] and body["where"] == "robots"
