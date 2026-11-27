"""webui/serve.py's fleet replay routes (fleetsim/replay.py): the Tests tab's
run list with its benches, one bench's runs, and one run packed, by a run's
name or bench/<bench>/<run>. Read without --control, so nothing a request
names may lead outside data/fleetsim/. No ROS, no sim; one short fleetsim
run recorded into a temp dir.

    /usr/bin/python3 -m pytest tests/fleetsim_replay_routes_test.py
"""
from __future__ import annotations

import gzip
import json
import shutil
import sys
import threading
import urllib.error
import urllib.parse
import urllib.request
from http.server import ThreadingHTTPServer
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT))
sys.path.insert(0, str(REPO_ROOT / "webui"))

import serve  # noqa: E402
from fleetsim import replay, sim  # noqa: E402
from fleetsim.tests.test_sim import scenario  # noqa: E402
from simtest import events as ev_lib  # noqa: E402

BENCH = "20261126T010203Z_smoke"
RUN = "delivery_smoke_r3_s1_radio"


@pytest.fixture(scope="module")
def results(tmp_path_factory):
    root = tmp_path_factory.mktemp("results")
    src = root / "plain"
    src.mkdir()
    scn = scenario(config={"network": {"model": "radio"}}, duration_s=20, stop={})
    (src / "scenario.json").write_text(json.dumps(scn, default=str))
    rec = ev_lib.Recorder(src / "events.jsonl.gz")
    try:
        summary = sim.Sim(scn, recorder=rec).run()
    finally:
        rec.close()
    (src / "summary.json").write_text(json.dumps(summary))
    b = root / "bench" / BENCH
    shutil.copytree(src, b / RUN)
    (b / "summary.json").write_text(json.dumps({
        "complete": True, "counts": {"pass": 1}, "suite": {"name": "smoke"},
        "runs": [{"name": RUN, "variant": "radio", "status": "pass"}]}))
    outside = tmp_path_factory.mktemp("outside")
    shutil.copytree(src, outside / "secret")
    (b / "linked_out").symlink_to(outside / "secret")
    return root


@pytest.fixture
def server(results, monkeypatch):
    monkeypatch.setattr(replay, "RESULTS_DIR", results)
    replay._cache.clear()
    srv = ThreadingHTTPServer(("127.0.0.1", 0), serve.DashboardHandler)
    threading.Thread(target=srv.serve_forever, daemon=True).start()

    def get(route, **q):
        url = f"http://127.0.0.1:{srv.server_port}{route}"
        if q:
            url += "?" + urllib.parse.urlencode(q)
        req = urllib.request.Request(url, headers={"Accept-Encoding": "gzip"})
        try:
            with urllib.request.urlopen(req, timeout=30) as r:
                body = r.read()
                if r.headers.get("Content-Encoding") == "gzip":
                    body = gzip.decompress(body)
                return r.status, json.loads(body)
        except urllib.error.HTTPError as e:
            return e.code, json.loads(e.read())

    yield get
    srv.shutdown()
    srv.server_close()


def test_the_list_has_the_runs_and_the_benches_closed(server):
    code, body = server("/api/fleetsim/runs")
    assert code == 200
    assert [r["dir"] for r in body["runs"]] == ["plain"]
    assert [(b["dir"], b["suite"], b["runs"]) for b in body["benches"]] == [(BENCH, "smoke", 1)]


def test_a_bench_lists_its_runs_by_reference(server):
    code, body = server("/api/fleetsim/bench", dir=BENCH)
    assert code == 200
    assert [(r["dir"], r["variant"]) for r in body["runs"]] == [(f"bench/{BENCH}/{RUN}", "radio")]


def test_a_bench_run_replays_with_its_monitor(server):
    code, body = server("/api/fleetsim/run", dir=f"bench/{BENCH}/{RUN}")
    assert code == 200
    assert body["dir"] == f"bench/{BENCH}/{RUN}" and body["network"] == "radio"
    c = body["comms"]
    assert c["rows"] and c["nodes"][3][1] and len(c["nodes"][3][1]) == 4


@pytest.mark.parametrize("ref", ["../plain", "bench/../plain", f"bench/{BENCH}/../plain",
                                 f"bench/{BENCH}", "bench", f"x/{BENCH}/{RUN}", "/etc/passwd"])
def test_a_path_is_refused(server, ref):
    code, body = server("/api/fleetsim/run", dir=ref)
    assert code == 400 and "bench/<bench>/<run>" in body["error"]


def test_a_link_out_of_the_results_dir_is_not_found_and_says_no_path(server, results):
    code, body = server("/api/fleetsim/run", dir=f"bench/{BENCH}/linked_out")
    assert code == 404 and str(results) not in body["error"]
    code, body = server("/api/fleetsim/bench", dir="../outside")
    assert code == 400
    code, body = server("/api/fleetsim/bench", dir="nope")
    assert code == 404 and str(results) not in body["error"]
