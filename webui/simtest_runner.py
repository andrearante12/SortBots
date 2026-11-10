"""Runs one scored test case at a time for the dashboard's Tests view.

The work itself is scripts/simtest.py — this only launches it detached, tails
its log, and cancels it. Same containment rules as webui/session.py: the test
NAME must match simtest.spec.NAME_RX and be a file actually in configs/tests/
(and validate) before it reaches argv, `repeats` is a range-checked int, a
variant label must be one the test declares, and Popen always gets a list.

A test run drives scripts/sim_ctl.sh, which starts sessions through this same
server's /api/session/start — so a running test is an ordinary session: the
live view shows it, actors included, while it is being scored.

Cancel is SIGINT to the process group: simtest.py's per-run `finally` stops
the sim and the probe scores what it saw (stop_reason: interrupted), so a
cancelled run still tears down cleanly and leaves a report.
"""
from __future__ import annotations

import os
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT))
from simtest import spec  # noqa: E402

LOG = spec.RESULTS_DIR / "_runner.log"
MAX_LOG_CHUNK = 64 * 1024


class SimtestRunner:
    def __init__(self, env_fn):
        self._env_fn = env_fn  # session.clean_env — strips ROS from the env
        self._lock = threading.Lock()
        self._proc: subprocess.Popen | None = None
        self._name: str | None = None
        self._started: float | None = None
        self._exit: int | None = None

    def running(self) -> bool:
        with self._lock:
            return self._proc is not None and self._proc.poll() is None

    def status(self, offset: int | None = None) -> dict:
        with self._lock:
            running = self._proc is not None and self._proc.poll() is None
            if self._proc is not None and not running and self._exit is None:
                self._exit = self._proc.returncode
            out = {"running": running, "name": self._name, "exit": self._exit,
                   "elapsed_s": round(time.monotonic() - self._started, 1)
                   if self._started else None}
        try:
            size = LOG.stat().st_size
            if offset is None or offset < 0:
                start = max(0, size - MAX_LOG_CHUNK)
            else:
                start = 0 if offset > size else offset  # log replaced by a new run
            with open(LOG, "rb") as f:
                f.seek(start)
                chunk = f.read(MAX_LOG_CHUNK)
            out.update(log=chunk.decode(errors="replace"), offset=start + len(chunk))
        except OSError:
            out.update(log="", offset=0)
        return out

    def start(self, name, repeats=None, variant=None) -> dict:
        if not isinstance(name, str) or not spec.NAME_RX.match(name):
            raise spec.SpecError("bad test name")
        if not spec.path_for(name).is_file():
            raise spec.SpecError(f"no test named {name!r}")
        t = spec.load(name)  # validate before launching anything
        argv = [sys.executable, str(REPO_ROOT / "scripts" / "simtest.py"), "run", name]
        if repeats is not None:
            if isinstance(repeats, bool) or not isinstance(repeats, int) \
                    or not 1 <= repeats <= spec.MAX_REPEATS:
                raise spec.SpecError(f"repeats must be 1..{spec.MAX_REPEATS}")
            argv += ["--repeats", str(repeats)]
        if variant is not None:
            labels = [label for label, _ in t["variants"]]
            if variant not in labels:
                raise spec.SpecError(f"variant must be one of {labels}")
            argv += ["--variant", variant]
        with self._lock:
            if self._proc is not None and self._proc.poll() is None:
                raise RuntimeError(f"test {self._name!r} is already running")
            LOG.parent.mkdir(parents=True, exist_ok=True)
            fh = open(LOG, "wb")
            try:
                self._proc = subprocess.Popen(
                    argv, cwd=str(REPO_ROOT), env=self._env_fn(), stdout=fh,
                    stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL,
                    start_new_session=True)
            finally:
                fh.close()
            self._name, self._started, self._exit = name, time.monotonic(), None
        return self.status()

    def cancel(self) -> dict:
        with self._lock:
            proc = self._proc
        if proc is not None and proc.poll() is None:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            except (ProcessLookupError, PermissionError):
                pass
        return self.status()
