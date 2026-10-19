#!/usr/bin/env python3
"""Splat worker: turns a saved map into a Gaussian splat. Runs on the WORKSTATION.

    scripts/splat.sh serve            # this, detached, in the `splat` conda env
    scripts/splat.sh build NAME [--from http://jetson-orin.tail0d28f9.ts.net:8081]

One job = one saved map (maps/<name>/), through these phases:

    queued -> fetching -> extracting -> building -> [waiting_for_gpu] -> training -> done
                                                          (any phase) -> failed | cancelled

  fetching    source "local": the workstation's own maps/<name>/*.db, read in
              place. Otherwise source is a dashboard URL (the Jetson's
              serve.py): GET /api/maps for the manifest, then each robot's DB
              from GET /api/maps/<name>/db?robot=<rid>, sha256-checked against
              the manifest. This is the ONLY thing that ever leaves the robot.
  extracting  rtabmap-export per robot DB (splat/extract.py).
  building    keyframe set in the fleet `map` frame (splat/dataset.py).
  training    splat/train.py as a subprocess (an OOM or CUDA fault must not take
              the server down); progress comes from its progress.json.

Sim and real are the same pipeline — a sim map is just "local". The dashboard
(served by EITHER host) never stores a splat: its browser fetches
GET /splats/<map>/latest.splat from here, cross-origin, hence CORS.

HTTP (default :8082):
  GET  /api/health                  {ok, busy, isaac_running, free_gb}
  GET  /api/jobs                    newest first
  GET  /api/jobs/<id>               one job
  GET  /api/jobs/<id>/log?offset=N  log tail, same shape as serve.py's session log
  POST /api/jobs                    {map, source?="local", iters?, force?}
  POST /api/jobs/<id>/cancel
  GET  /api/splats                  finished models, newest first
  GET  /splats/<map>/<job|latest>.splat

SECURITY: like serve.py --control, this binds 0.0.0.0 with no auth and Tailscale
ACLs are the perimeter. A POST can make it download from a `source` URL, but
only from fixed /api/maps paths on that host, only into its own cache dir, and
map names/job ids are regex-confined before they ever touch a path.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import shutil
import signal
import subprocess
import sys
import threading
import time
import urllib.error
import urllib.parse
import urllib.request
from datetime import datetime, timezone
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import yaml

HERE = Path(__file__).resolve().parent
REPO_ROOT = HERE.parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(REPO_ROOT / "scripts"))
import dataset  # noqa: E402
import extract  # noqa: E402
import maps_lib  # noqa: E402

CONFIG = REPO_ROOT / "configs" / "splat.yaml"
NAME_RX = maps_lib.NAME_RX
JOB_RX = re.compile(r"^[a-z0-9][a-z0-9_-]{0,63}-\d{8}T\d{6}Z$")
TERMINAL = ("done", "failed", "cancelled")
MAX_BODY_BYTES = 64 * 1024
MAX_LOG_CHUNK = 64 * 1024

# `[s]` so the pattern never matches a command line that merely CONTAINS the
# pattern — a plain `pgrep -f spawn_warehouse.py` run from a shell matches that
# shell itself and reports Isaac up when it isn't (bit us 2026-10-10).
ISAAC_PATTERN = "[s]pawn_warehouse.py"
GPU_POLL_S = 10.0

# Free space a job needs beyond the DBs it downloads: export (~0.5 GB for the
# warehouse) + keyframe set (hardlinked, ~free) + models. The disk on the
# workstation ran dry mid-session once already; fail at the door instead.
HEADROOM_BYTES = 2 * 1024 ** 3


class JobError(RuntimeError):
    """A job failed for a reason worth showing the operator verbatim."""


class Cancelled(Exception):
    pass


def utcstamp() -> str:
    return datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")


def load_config(path: Path = CONFIG) -> dict:
    with open(path) as f:
        return yaml.safe_load(f)


def splat_root(cfg: dict) -> Path:
    raw = os.environ.get("SORTBOTS_SPLAT_DIR") or cfg["worker"]["dir"]
    return Path(os.path.expanduser(raw))


def isaac_running() -> bool:
    try:
        return subprocess.run(["pgrep", "-f", ISAAC_PATTERN], stdout=subprocess.DEVNULL,
                              stderr=subprocess.DEVNULL).returncode == 0
    except OSError:
        return False


def normalize_source(source) -> str:
    """'local' or an http(s) base URL with no path. Raises JobError otherwise."""
    if source in (None, "", "local"):
        return "local"
    if not isinstance(source, str):
        raise JobError("source must be 'local' or a dashboard URL")
    u = urllib.parse.urlsplit(source.strip())
    if u.scheme not in ("http", "https") or not u.hostname:
        raise JobError(f"source {source!r} is not an http(s) URL")
    return f"{u.scheme}://{u.netloc}"


# --------------------------------------------------------------------------
# Fetching
# --------------------------------------------------------------------------

def _http_json(url: str, timeout: float = 20) -> dict:
    with urllib.request.urlopen(url, timeout=timeout) as r:
        return json.loads(r.read())


def remote_manifest(base: str, name: str) -> tuple[dict, str]:
    """(manifest, platform) for map `name` on the dashboard at `base`."""
    try:
        listing = _http_json(f"{base}/api/maps")
    except (urllib.error.URLError, OSError, ValueError) as e:
        raise JobError(f"can't reach {base}/api/maps: {e}") from e
    for m in listing.get("maps", []):
        if m.get("name") == name:
            platform = listing.get("platform")
            if platform is None:
                # Older serve.py: /api/maps has no platform; the launch form does.
                try:
                    platform = _http_json(f"{base}/api/launch/options").get("platform")
                except (urllib.error.URLError, OSError, ValueError):
                    platform = None
            return m, platform or "sim"
    raise JobError(f"no map named {name!r} on {base}")


def download(url: str, dst: Path, expect_sha256: str | None, should_stop, log) -> None:
    tmp = dst.with_suffix(dst.suffix + ".part")
    h = hashlib.sha256()
    done = 0
    t0 = time.monotonic()
    try:
        with urllib.request.urlopen(url, timeout=60) as r, open(tmp, "wb") as f:
            total = int(r.headers.get("Content-Length") or 0)
            while True:
                if should_stop():
                    raise Cancelled()
                chunk = r.read(1 << 20)
                if not chunk:
                    break
                f.write(chunk)
                h.update(chunk)
                done += len(chunk)
                if done % (256 << 20) < (1 << 20):
                    rate = done / max(time.monotonic() - t0, 1e-3) / 1e6
                    log(f"  {done / 1e9:.2f}/{total / 1e9:.2f} GB ({rate:.0f} MB/s)")
    except urllib.error.HTTPError as e:
        tmp.unlink(missing_ok=True)
        detail = e.read()[:300].decode("utf-8", "replace")
        raise JobError(f"GET {url}: HTTP {e.code} {detail}") from e
    except (urllib.error.URLError, OSError) as e:
        tmp.unlink(missing_ok=True)
        raise JobError(f"GET {url}: {e}") from e
    if expect_sha256 and h.hexdigest() != expect_sha256:
        tmp.unlink(missing_ok=True)
        raise JobError(f"{dst.name}: sha256 mismatch (torn transfer?)")
    os.replace(tmp, dst)


def anchor_scene(manifest: dict, platform: str) -> str:
    # A robot's map is anchored at spawn.real, whatever the manifest says: the
    # dashboard's save doesn't pass --scene, so maps.sh records its default
    # "nvidia" even on the Jetson. Same rule bringup applies ("Forced rather
    # than trusted"): platform real => scene real.
    return "real" if platform == "real" else (manifest.get("scene") or "nvidia")


# --------------------------------------------------------------------------
# Jobs
# --------------------------------------------------------------------------

class Job:
    def __init__(self, root: Path, map_name: str, source: str, iters: int | None,
                 force: bool, job_id: str | None = None):
        self.id = job_id or f"{map_name}-{utcstamp()}"
        self.map = map_name
        self.source = source
        self.iters = iters
        self.force = force
        self.dir = root / map_name / self.id
        self.state = "queued"
        self.error: str | None = None
        self.created = time.time()
        self.finished: float | None = None
        self.result: dict = {}
        self.cancel_requested = False
        self.proc: subprocess.Popen | None = None

    @property
    def log_path(self) -> Path:
        return self.dir / "job.log"

    def to_dict(self) -> dict:
        d = {
            "id": self.id, "map": self.map, "source": self.source, "state": self.state,
            "error": self.error, "created": self.created, "finished": self.finished,
            "iters": self.iters, "result": self.result,
        }
        prog = self.dir / "progress.json"
        if self.state == "training" and prog.is_file():
            try:
                d["progress"] = json.loads(prog.read_text())
            except (OSError, ValueError):
                pass
        if (self.dir / "model.splat").is_file():
            d["splat_url"] = f"/splats/{self.map}/{self.id}.splat"
        return d

    def persist(self) -> None:
        self.dir.mkdir(parents=True, exist_ok=True)
        tmp = self.dir / ".job.json.tmp"
        tmp.write_text(json.dumps(self.to_dict(), indent=1))
        os.replace(tmp, self.dir / "job.json")


class Worker:
    """One job at a time, FIFO. The GPU is the bottleneck; parallel jobs would
    only fight over it (and over 8 GB of VRAM)."""

    def __init__(self, cfg: dict, root: Path | None = None, *,
                 isaac_check=isaac_running, train_argv=None):
        self.cfg = cfg
        self.root = root or splat_root(cfg)
        self.root.mkdir(parents=True, exist_ok=True)
        self.isaac_check = isaac_check
        self.train_argv = train_argv or self._default_train_argv
        self.jobs: dict[str, Job] = {}
        self.queue: list[Job] = []
        self.current: Job | None = None
        self.cv = threading.Condition()
        self._load_history()

    # -- history ---------------------------------------------------------

    def _load_history(self) -> None:
        """Finished jobs from disk, so /api/splats survives a restart. A job
        that was mid-flight when the worker died is reported failed."""
        for p in sorted(self.root.glob("*/*/job.json")):
            try:
                d = json.loads(p.read_text())
            except (OSError, ValueError):
                continue
            if not (JOB_RX.match(str(d.get("id", ""))) and NAME_RX.match(str(d.get("map", "")))):
                continue
            j = Job(self.root, d["map"], d.get("source", "local"), d.get("iters"), False,
                    job_id=d["id"])
            j.created = d.get("created") or p.stat().st_mtime
            j.finished = d.get("finished")
            j.result = d.get("result") or {}
            j.state = d.get("state", "failed")
            j.error = d.get("error")
            if j.state not in TERMINAL:
                j.state, j.error = "failed", "worker restarted mid-job"
                j.finished = j.finished or p.stat().st_mtime
            self.jobs[j.id] = j

    # -- API -------------------------------------------------------------

    def submit(self, map_name, source="local", iters=None, force=False) -> Job:
        if not isinstance(map_name, str) or not NAME_RX.match(map_name):
            raise JobError(f"map name {map_name!r} does not match {NAME_RX.pattern}")
        source = normalize_source(source)
        if iters is not None:
            if not isinstance(iters, int) or not (100 <= iters <= 100_000):
                raise JobError("iters must be an integer in [100, 100000]")
        with self.cv:
            for j in [self.current, *self.queue]:
                if j is not None and j.map == map_name and j.source == source:
                    raise JobError(f"{map_name} is already queued or building ({j.id})")
            job = Job(self.root, map_name, source, iters, bool(force))
            while job.id in self.jobs:   # two submits in the same second
                time.sleep(1.0)
                job = Job(self.root, map_name, source, iters, bool(force))
            self.jobs[job.id] = job
            self.queue.append(job)
            job.persist()
            self.cv.notify_all()
        return job

    def cancel(self, job_id: str) -> Job:
        with self.cv:
            job = self.jobs.get(job_id)
            if job is None:
                raise KeyError(job_id)
            if job.state in TERMINAL:
                return job
            job.cancel_requested = True
            if job in self.queue:
                self.queue.remove(job)
                self._finish(job, "cancelled")
            elif job.proc is not None and job.proc.poll() is None:
                # train.py exports what it has on SIGTERM; we still mark it
                # cancelled, but the partial model.splat stays viewable.
                job.proc.send_signal(signal.SIGTERM)
            self.cv.notify_all()
        return job

    def list_jobs(self) -> list[dict]:
        return [j.to_dict() for j in sorted(self.jobs.values(), key=lambda j: -j.created)]

    def splats(self) -> list[dict]:
        out = []
        for j in sorted(self.jobs.values(), key=lambda j: -j.created):
            f = j.dir / "model.splat"
            if j.state == "done" and f.is_file():
                out.append({"map": j.map, "job": j.id, "source": j.source,
                            "finished": j.finished, "bytes": f.stat().st_size,
                            "gaussians": f.stat().st_size // 32,
                            "psnr": j.result.get("psnr"),
                            "url": f"/splats/{j.map}/{j.id}.splat"})
        return out

    def splat_file(self, map_name: str, which: str) -> Path | None:
        if not NAME_RX.match(map_name):
            return None
        if which == "latest":
            for s in self.splats():
                if s["map"] == map_name:
                    return self.jobs[s["job"]].dir / "model.splat"
            return None
        job = self.jobs.get(which)
        if job is None or job.map != map_name:
            return None
        f = job.dir / "model.splat"
        # A running job's snapshot is fine to view; that's what snapshots are for.
        return f if f.is_file() else None

    def health(self) -> dict:
        return {"ok": True, "busy": self.current is not None, "queued": len(self.queue),
                "isaac_running": self.isaac_check(),
                "free_gb": round(shutil.disk_usage(self.root).free / 1e9, 1)}

    # -- running ---------------------------------------------------------

    def run_forever(self) -> None:
        while True:
            with self.cv:
                while not self.queue:
                    self.cv.wait()
                job = self.queue.pop(0)
                self.current = job
            try:
                self.run_job(job)
            finally:
                with self.cv:
                    self.current = None

    def _finish(self, job: Job, state: str, error: str | None = None) -> None:
        job.state, job.error, job.finished = state, error, time.time()
        job.persist()

    def _set(self, job: Job, state: str) -> None:
        if job.cancel_requested:
            raise Cancelled()
        job.state = state
        job.persist()

    def run_job(self, job: Job) -> None:
        job.dir.mkdir(parents=True, exist_ok=True)
        with open(job.log_path, "a", buffering=1) as logf:
            def log(msg: str) -> None:
                logf.write(f"[{time.strftime('%H:%M:%S')}] {msg}\n")

            try:
                self._run_phases(job, log, logf)
                self._finish(job, "cancelled" if job.cancel_requested else "done")
                log(f"{job.state}: {job.result}")
            except Cancelled:
                log("cancelled")
                self._finish(job, "cancelled")
            except (JobError, dataset.DatasetError, maps_lib.MapError, RuntimeError,
                    OSError, subprocess.SubprocessError) as e:
                log(f"FAILED: {e}")
                self._finish(job, "failed", str(e))
            finally:
                self._cleanup(job, keep=self.cfg["worker"].get("keep_dataset", False))

    def _run_phases(self, job: Job, log, logf) -> None:
        dcfg = self.cfg.get("dataset", {})
        db_dir = job.dir / "db"
        export_dir = job.dir / "export"
        ds_dir = job.dir / "dataset"

        # -- fetching ----------------------------------------------------
        self._set(job, "fetching")
        if job.source == "local":
            manifest = maps_lib.read_manifest(job.map)
            platform = maps_lib.detect_platform()
            base_dir = maps_lib.MAPS_DIR / job.map
            log(f"local map {job.map} ({maps_lib.MAPS_DIR})")
        else:
            manifest, platform = remote_manifest(job.source, job.map)
            base_dir = db_dir
            log(f"remote map {job.map} on {job.source} (platform {platform})")
        if manifest.get("db_state") != "complete":
            raise JobError(f"map {job.map} db_state is {manifest.get('db_state')!r}, "
                           "not complete — save it again after the run stops")
        dbs = manifest.get("dbs") or {}
        if not dbs:
            raise JobError(f"map {job.map} has no pose graph")

        need = HEADROOM_BYTES + (sum(int(e.get("bytes") or 0) for e in dbs.values())
                                 if job.source != "local" else 0)
        free = shutil.disk_usage(self.root).free
        if free < need:
            raise JobError(f"only {free / 1e9:.1f} GB free under {self.root}, "
                           f"need ~{need / 1e9:.1f} GB")

        db_files: dict[str, Path] = {}
        for rid, entry in dbs.items():
            fname = Path(entry["file"]).name
            if job.source == "local":
                db_files[rid] = base_dir / fname
            else:
                db_dir.mkdir(parents=True, exist_ok=True)
                url = (f"{job.source}/api/maps/{urllib.parse.quote(job.map)}/db?"
                       f"robot={urllib.parse.quote(rid)}")
                log(f"downloading {rid} pose graph ({int(entry.get('bytes') or 0) / 1e9:.2f} GB)")
                download(url, db_dir / fname, entry.get("sha256"),
                         lambda: job.cancel_requested, log)
                db_files[rid] = db_dir / fname

        # -- extracting --------------------------------------------------
        self._set(job, "extracting")
        scene = anchor_scene(manifest, platform)
        sources = []
        for rid, db in db_files.items():
            log(f"rtabmap-export {rid}: {db}")
            extract.run_export(db, export_dir, rid, logf)
            sources.append({"export_dir": export_dir, "prefix": rid, "robot_id": rid,
                            "anchor": dataset.spawn_anchor(scene, rid)})
        # The downloaded DBs are the biggest thing in the job; the export has
        # everything training needs.
        shutil.rmtree(db_dir, ignore_errors=True)

        # -- building ----------------------------------------------------
        self._set(job, "building")
        t = dataset.build_dataset(
            sources, ds_dir,
            init_points=int(dcfg.get("init_points", dataset.DEFAULT_INIT_POINTS)),
            init_voxel=float(dcfg.get("init_voxel", dataset.DEFAULT_INIT_VOXEL)),
            stride=int(dcfg.get("stride", 1)),
            meta={"map": job.map, "source": job.source, "scene": scene},
        )
        log(f"keyframe set: {len(t['frames'])} frames, {t['init_points']} init points, "
            f"anchored with spawn.{scene}")
        shutil.rmtree(export_dir, ignore_errors=True)   # frames are hardlinked

        # -- training ----------------------------------------------------
        if self.cfg["worker"].get("wait_for_isaac", True) and not job.force:
            while self.isaac_check():
                if job.state != "waiting_for_gpu":
                    log("Isaac Sim is running on this GPU; waiting (force=true skips)")
                    self._set(job, "waiting_for_gpu")
                if job.cancel_requested:
                    raise Cancelled()
                with self.cv:
                    self.cv.wait(GPU_POLL_S)
        self._set(job, "training")
        argv = self.train_argv(ds_dir, job)
        log("$ " + " ".join(map(str, argv)))
        job.proc = subprocess.Popen(argv, stdout=logf, stderr=subprocess.STDOUT,
                                    cwd=str(REPO_ROOT))
        rc = job.proc.wait()
        job.proc = None
        prog = job.dir / "progress.json"
        if prog.is_file():
            try:
                job.result = json.loads(prog.read_text())
            except ValueError:
                pass
        if rc != 0 and not job.cancel_requested:
            raise JobError(f"train.py exited {rc} (see job log)")
        if not (job.dir / "model.splat").is_file():
            raise JobError("training finished without writing model.splat")

    def _default_train_argv(self, ds_dir: Path, job: Job) -> list[str]:
        # sys.executable: the worker itself runs in the `splat` env.
        argv = [sys.executable, str(HERE / "train.py"), str(ds_dir), str(job.dir)]
        if job.iters:
            argv += ["--iters", str(job.iters)]
        return argv

    @staticmethod
    def _cleanup(job: Job, keep: bool) -> None:
        shutil.rmtree(job.dir / "db", ignore_errors=True)
        shutil.rmtree(job.dir / "export", ignore_errors=True)
        if not keep or job.state != "done":
            shutil.rmtree(job.dir / "dataset", ignore_errors=True)


# --------------------------------------------------------------------------
# HTTP
# --------------------------------------------------------------------------

WORKER: Worker | None = None


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def end_headers(self):
        # The dashboard page comes from serve.py on :8081 (possibly another
        # machine — the Jetson); every request here is cross-origin.
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.send_header("Cache-Control", "no-cache")
        super().end_headers()

    def do_OPTIONS(self):
        self.send_response(204)
        self.send_header("Content-Length", "0")
        self.end_headers()

    def do_GET(self):
        u = urllib.parse.urlsplit(self.path)
        parts = [p for p in u.path.split("/") if p]
        if u.path == "/api/health":
            self._json(WORKER.health())
        elif u.path == "/api/jobs":
            self._json({"jobs": WORKER.list_jobs()})
        elif u.path == "/api/splats":
            self._json({"splats": WORKER.splats()})
        elif len(parts) == 3 and parts[:2] == ["api", "jobs"]:
            job = WORKER.jobs.get(parts[2])
            self._json(job.to_dict()) if job else self._error(404, "no such job")
        elif len(parts) == 4 and parts[:2] == ["api", "jobs"] and parts[3] == "log":
            self._log(parts[2], u.query)
        elif len(parts) == 3 and parts[0] == "splats" and parts[2].endswith(".splat"):
            self._splat(parts[1], parts[2][:-len(".splat")])
        else:
            self._error(404, f"no such endpoint: {u.path}")

    def do_POST(self):
        u = urllib.parse.urlsplit(self.path)
        parts = [p for p in u.path.split("/") if p]
        body = self._body()
        if body is None:
            return
        if u.path == "/api/jobs":
            try:
                job = WORKER.submit(body.get("map"), body.get("source") or "local",
                                    body.get("iters"), bool(body.get("force")))
            except JobError as e:
                self._error(400, str(e))
                return
            self._json(job.to_dict(), 201)
        elif len(parts) == 4 and parts[:2] == ["api", "jobs"] and parts[3] == "cancel":
            try:
                self._json(WORKER.cancel(parts[2]).to_dict())
            except KeyError:
                self._error(404, "no such job")
        else:
            self._error(404, f"no such endpoint: {u.path}")

    def _log(self, job_id: str, query: str):
        job = WORKER.jobs.get(job_id)
        if job is None:
            self._error(404, "no such job")
            return
        raw = urllib.parse.parse_qs(query).get("offset", ["0"])[0]
        try:
            offset = max(0, int(raw))
        except ValueError:
            self._error(400, "offset must be an integer")
            return
        text, size = "", 0
        if job.log_path.is_file():
            size = job.log_path.stat().st_size
            with open(job.log_path, "rb") as f:
                f.seek(min(offset, size))
                text = f.read(MAX_LOG_CHUNK).decode("utf-8", "replace")
        self._json({"offset": min(offset, size) + len(text.encode()), "size": size,
                    "text": text, "state": job.state})

    def _splat(self, map_name: str, which: str):
        f = WORKER.splat_file(map_name, which)
        if f is None:
            self._error(404, f"no splat for {map_name}/{which}")
            return
        size = f.stat().st_size
        self.send_response(200)
        self.send_header("Content-Type", "application/octet-stream")
        self.send_header("Content-Length", str(size))
        self.end_headers()
        with open(f, "rb") as fh:
            shutil.copyfileobj(fh, self.wfile, 1 << 20)

    def _body(self):
        try:
            n = int(self.headers.get("Content-Length") or 0)
        except ValueError:
            self._error(400, "bad Content-Length")
            return None
        if n > MAX_BODY_BYTES:
            self._error(413, "request body too large")
            return None
        try:
            body = json.loads(self.rfile.read(n) or b"{}") if n else {}
        except ValueError:
            self._error(400, "body must be JSON")
            return None
        if not isinstance(body, dict):
            self._error(400, "body must be a JSON object")
            return None
        return body

    def _json(self, obj, code: int = 200):
        data = json.dumps(obj).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def _error(self, code: int, msg: str):
        self._json({"error": msg}, code)

    def log_message(self, fmt, *args):
        pass


def main(argv=None) -> int:
    global WORKER
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int)
    ap.add_argument("--config", type=Path, default=CONFIG)
    args = ap.parse_args(argv)
    cfg = load_config(args.config)
    WORKER = Worker(cfg)
    threading.Thread(target=WORKER.run_forever, daemon=True, name="splat-jobs").start()
    port = args.port or int(cfg["worker"].get("port", 8082))
    server = ThreadingHTTPServer((args.host, port), Handler)
    server.daemon_threads = True
    print(f"SortBots splat worker: http://{args.host}:{port}/ (artifacts in {WORKER.root})",
          flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
