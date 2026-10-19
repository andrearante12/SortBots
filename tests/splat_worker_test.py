"""splat/worker.py job flow, and serve.py's map-download side of it.

No GPU, no ROS, no torch: rtabmap-export is replaced by a synthetic export
(tests/splat_helpers.py) and train.py by a tiny script that writes what the
real one does. The remote path runs against a REAL webui/serve.py subprocess
with a seeded map library — the same pull the workstation does from the Jetson.
"""
import hashlib
import json
import os
import socket
import subprocess
import sys
import threading
import time
import urllib.error
import urllib.request
from http.server import ThreadingHTTPServer
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "splat"))
sys.path.insert(0, str(Path(__file__).resolve().parent))

import worker as W  # noqa: E402  (first: it puts scripts/ on sys.path)
import extract  # noqa: E402
import maps_lib  # noqa: E402
from splat_helpers import make_export  # noqa: E402

DB_BYTES = b"SQLite format 3\0" + bytes(4080)

FAKE_TRAIN = """
import json, sys, pathlib
out = pathlib.Path(sys.argv[2])
t = json.loads((pathlib.Path(sys.argv[1]) / "transforms.json").read_text())
(out / "model.splat").write_bytes(bytes(32 * len(t["frames"])))
(out / "progress.json").write_text(json.dumps({"step": 10, "iters": 10, "psnr": 30.0,
                                               "gaussians": len(t["frames"])}))
"""


def seed_map(maps_dir: Path, name: str, *, state="complete", robots=("robot_0",)):
    d = maps_dir / name
    d.mkdir(parents=True)
    dbs = {}
    for i, rid in enumerate(robots):
        f = "map.db" if i == 0 else f"map_{rid}.db"
        (d / f).write_bytes(DB_BYTES)
        dbs[rid] = {"file": f, "bytes": len(DB_BYTES), "state": state,
                    "sha256": hashlib.sha256(DB_BYTES).hexdigest()}
    (d / "map.json").write_text(json.dumps({
        "schema": 1, "name": name, "title": name, "description": "",
        "created": "2026-10-10T00:00:00Z", "updated": "2026-10-10T00:00:00Z",
        "scene": "nvidia", "robot_ids": list(robots), "primary_robot_id": robots[0],
        "source": {}, "grid": None, "coverage": None, "dbs": dbs,
        "db_state": state, "versions": {},
    }))


@pytest.fixture
def env(tmp_path, monkeypatch):
    maps_dir = tmp_path / "maps"
    maps_dir.mkdir()
    monkeypatch.setattr(maps_lib, "MAPS_DIR", maps_dir)
    monkeypatch.setattr(W, "GPU_POLL_S", 0.01)
    monkeypatch.setattr(W, "HEADROOM_BYTES", 0)
    exports = []

    def fake_export(db, out_dir, prefix, log, timeout=3600):
        assert Path(db).read_bytes() == DB_BYTES       # local or downloaded, intact
        exports.append((Path(db), prefix))
        make_export(out_dir, prefix, {1: (0, 0, 1.2), 2: (1, 0, 1.2)})

    monkeypatch.setattr(extract, "run_export", fake_export)
    script = tmp_path / "fake_train.py"
    script.write_text(FAKE_TRAIN)
    cfg = {"worker": {"keep_dataset": False, "wait_for_isaac": True},
           "dataset": {"init_points": 1000}}

    def make(isaac=lambda: False):
        return W.Worker(cfg, tmp_path / "splats", isaac_check=isaac,
                        train_argv=lambda ds, job: [sys.executable, str(script), str(ds), str(job.dir)])

    return {"maps": maps_dir, "make": make, "exports": exports, "root": tmp_path / "splats"}


def run_next(worker):
    job = worker.queue.pop(0)
    worker.run_job(job)
    return job


def test_local_job_end_to_end(env):
    seed_map(env["maps"], "warehouse")
    w = env["make"]()
    w.submit("warehouse")
    job = run_next(w)
    assert job.state == "done", job.error
    assert job.result["psnr"] == 30.0
    assert (job.dir / "model.splat").stat().st_size == 64
    # Local DB read in place, not copied.
    assert env["exports"] == [(env["maps"] / "warehouse" / "map.db", "robot_0")]
    # Intermediates cleaned up; the model and log stay.
    assert not (job.dir / "export").exists() and not (job.dir / "dataset").exists()
    assert (job.dir / "job.log").is_file()
    assert [s["job"] for s in w.splats()] == [job.id]
    assert w.splat_file("warehouse", "latest") == job.dir / "model.splat"
    assert w.splat_file("warehouse", job.id) == job.dir / "model.splat"
    assert w.splat_file("other", job.id) is None


def test_fleet_map_exports_every_robot(env):
    seed_map(env["maps"], "fleet", robots=("robot_0", "robot_1"))
    w = env["make"]()
    w.submit("fleet")
    job = run_next(w)
    assert job.state == "done", job.error
    assert sorted(p for _, p in env["exports"]) == ["robot_0", "robot_1"]
    assert (job.dir / "model.splat").stat().st_size == 32 * 4


def test_refuses_pending_maps_and_bad_input(env):
    seed_map(env["maps"], "half", state="pending")
    w = env["make"]()
    w.submit("half")
    job = run_next(w)
    assert job.state == "failed" and "pending" in job.error
    for bad in ("../etc", "Warehouse", "", None):
        with pytest.raises(W.JobError):
            w.submit(bad)
    with pytest.raises(W.JobError):
        w.submit("half", iters=5)
    with pytest.raises(W.JobError):
        w.submit("half", source="file:///etc/passwd")


def test_duplicate_submit_rejected_while_queued(env):
    seed_map(env["maps"], "warehouse")
    w = env["make"]()
    w.submit("warehouse")
    with pytest.raises(W.JobError):
        w.submit("warehouse")


def test_waits_for_isaac_then_trains(env):
    seed_map(env["maps"], "warehouse")
    checks = iter([True, True, False])
    states = []
    w = env["make"](isaac=lambda: next(checks, False))
    orig = w._set
    w._set = lambda job, state: (states.append(state), orig(job, state))
    w.submit("warehouse")
    job = run_next(w)
    assert job.state == "done"
    assert states == ["fetching", "extracting", "building", "waiting_for_gpu", "training"]


def test_force_skips_the_isaac_wait(env):
    seed_map(env["maps"], "warehouse")
    w = env["make"](isaac=lambda: True)
    w.submit("warehouse", force=True)
    assert run_next(w).state == "done"


def test_cancel_queued_and_waiting(env):
    seed_map(env["maps"], "warehouse")
    w = env["make"]()
    job = w.submit("warehouse")
    assert w.cancel(job.id).state == "cancelled" and not w.queue

    w2 = env["make"](isaac=lambda: True)
    job2 = w2.submit("warehouse")
    w2.queue.pop(0)
    t = threading.Thread(target=w2.run_job, args=(job2,))
    t.start()
    for _ in range(200):
        if job2.state == "waiting_for_gpu":
            break
        time.sleep(0.01)
    w2.cancel(job2.id)
    t.join(5)
    assert job2.state == "cancelled"


def test_history_survives_restart(env):
    seed_map(env["maps"], "warehouse")
    w = env["make"]()
    w.submit("warehouse")
    done = run_next(w)
    # A job that was mid-flight when the worker died.
    w.submit("warehouse")
    stuck = w.queue.pop(0)
    stuck.state = "training"
    stuck.persist()
    w2 = env["make"]()
    assert w2.jobs[done.id].state == "done"
    assert w2.jobs[stuck.id].state == "failed"
    assert "restarted" in w2.jobs[stuck.id].error
    assert [s["job"] for s in w2.splats()] == [done.id]


def test_anchor_scene_forces_real_on_the_robot():
    # maps.sh records scene "nvidia" by default even on the Jetson.
    assert W.anchor_scene({"scene": "nvidia"}, "real") == "real"
    assert W.anchor_scene({"scene": "primitive"}, "sim") == "primitive"
    assert W.anchor_scene({}, "sim") == "nvidia"


def test_normalize_source():
    assert W.normalize_source(None) == "local"
    assert W.normalize_source("local") == "local"
    assert W.normalize_source("http://jetson-orin:8081/some/page?x=1") == "http://jetson-orin:8081"
    for bad in ("ftp://x", "jetson:8081", "http://", 5):
        with pytest.raises(W.JobError):
            W.normalize_source(bad)


# -------------------------------------------------------------- remote source

def _free_port() -> int:
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


@pytest.fixture
def robot_dashboard(env):
    """A real webui/serve.py --platform real over the seeded library."""
    port = _free_port()
    proc = subprocess.Popen(
        [sys.executable, str(REPO / "webui" / "serve.py"), "--platform", "real",
         "--host", "127.0.0.1", "--port", str(port)],
        env={**os.environ, "SORTBOTS_MAPS_DIR": str(env["maps"])},
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    base = f"http://127.0.0.1:{port}"
    for _ in range(100):
        try:
            urllib.request.urlopen(base + "/api/maps", timeout=1)
            break
        except OSError:
            time.sleep(0.05)
    yield base
    proc.kill()
    proc.wait()


def _get(url):
    try:
        with urllib.request.urlopen(url, timeout=5) as r:
            return r.status, r.read()
    except urllib.error.HTTPError as e:
        return e.code, e.read()


def test_serve_map_db_endpoint(env, robot_dashboard):
    seed_map(env["maps"], "warehouse")
    seed_map(env["maps"], "half", state="pending")
    base = robot_dashboard
    code, body = _get(base + "/api/maps/warehouse/db")
    assert code == 200 and body == DB_BYTES
    assert _get(base + "/api/maps/warehouse/db?robot=robot_0")[0] == 200
    assert _get(base + "/api/maps/warehouse/db?robot=robot_9")[0] == 404
    assert _get(base + "/api/maps/half/db")[0] == 409
    assert _get(base + "/api/maps/nope/db")[0] == 404
    assert _get(base + "/api/maps/..%2F..%2Fetc/db")[0] == 404
    # The platform the worker anchors by, and the worker URL the page uses.
    assert json.loads(_get(base + "/api/maps")[1])["platform"] == "real"
    cfg = json.loads(_get(base + "/api/splat/config")[1])
    assert cfg["platform"] == "real" and cfg["worker_url"].startswith("http")


def test_remote_job_pulls_the_db_from_the_robot(env, robot_dashboard):
    seed_map(env["maps"], "warehouse")
    w = env["make"]()
    w.submit("warehouse", source=robot_dashboard)
    job = run_next(w)
    assert job.state == "done", job.error
    downloaded, prefix = env["exports"][0]
    assert downloaded.parent == job.dir / "db" and prefix == "robot_0"
    assert not (job.dir / "db").exists()            # deleted once exported
    meta = json.loads((job.dir / "job.json").read_text())
    assert meta["source"] == robot_dashboard
    log = (job.dir / "job.log").read_text()
    assert "platform real" in log and "spawn.real" in log


def test_remote_job_rejects_a_torn_download(env, robot_dashboard):
    seed_map(env["maps"], "warehouse")
    m = json.loads((env["maps"] / "warehouse" / "map.json").read_text())
    m["dbs"]["robot_0"]["sha256"] = "0" * 64
    (env["maps"] / "warehouse" / "map.json").write_text(json.dumps(m))
    w = env["make"]()
    w.submit("warehouse", source=robot_dashboard)
    job = run_next(w)
    assert job.state == "failed" and "sha256" in job.error
    assert not list((job.dir).glob("db/*"))


# -------------------------------------------------------------- HTTP surface

def test_http_cors_and_routes(env):
    seed_map(env["maps"], "warehouse")
    W.WORKER = env["make"]()
    W.WORKER.submit("warehouse")
    job = run_next(W.WORKER)
    srv = ThreadingHTTPServer(("127.0.0.1", 0), W.Handler)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    base = f"http://127.0.0.1:{srv.server_address[1]}"
    try:
        req = urllib.request.Request(base + "/api/jobs", method="OPTIONS")
        with urllib.request.urlopen(req) as r:
            assert r.status == 204
            assert r.headers["Access-Control-Allow-Origin"] == "*"
        with urllib.request.urlopen(base + "/api/splats") as r:
            assert r.headers["Access-Control-Allow-Origin"] == "*"
            assert json.loads(r.read())["splats"][0]["job"] == job.id
        code, body = _get(f"{base}/splats/warehouse/latest.splat")
        assert code == 200 and len(body) == 64
        assert _get(f"{base}/splats/warehouse/nope.splat")[0] == 404
        assert _get(f"{base}/splats/..%2Fx/latest.splat")[0] == 404
        code, body = _get(f"{base}/api/jobs/{job.id}/log?offset=0")
        assert code == 200 and "done" in json.loads(body)["text"]
        req = urllib.request.Request(base + "/api/jobs", data=b'{"map": "../x"}', method="POST",
                                     headers={"Content-Type": "application/json"})
        try:
            urllib.request.urlopen(req)
            raise AssertionError("expected 400")
        except urllib.error.HTTPError as e:
            assert e.code == 400
    finally:
        srv.shutdown()
