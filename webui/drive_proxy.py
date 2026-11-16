"""serve.py's half of the Drive view: starts scripts/drive_bridge.py and talks to it.

The bridge runs in the `lerobot` conda env (pyrealsense2, cv2, lerobot's servo bus) and
serve.py in the system python3, so they are two processes. They talk over a unix socket
(one JSON line a request), never a TCP port: serve.py stays the only HTTP server, and the
process that moves the robot is not reachable from the network by itself.

The bridge holds the servo bus and the D435 only while it runs, so it is started on the
operator's Connect and stopped on Disconnect, not with serve.py: the arm's own tools
(scripts/head.py, arm.sh pad) need the same board and camera the rest of the time.
"""
from __future__ import annotations

import json
import os
import socket
import subprocess
import tempfile
import threading
import time
from collections import deque
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
BRIDGE = REPO / "scripts" / "drive_bridge.py"
DEFAULT_PYTHON = Path.home() / "miniconda3" / "envs" / "lerobot" / "bin" / "python"
LOG_LINES = 40


class DriveProxy:
    def __init__(self, python: str | os.PathLike | None = None, config: str | None = None,
                 socket_path: str | None = None):
        self.python = str(python or os.environ.get("SORTBOTS_LEROBOT_PYTHON") or DEFAULT_PYTHON)
        # SORTBOTS_DRIVE_CONFIG: the tests point it at a config with no camera and no real port.
        self.config = config or os.environ.get("SORTBOTS_DRIVE_CONFIG")
        run_dir = os.environ.get("XDG_RUNTIME_DIR") or tempfile.gettempdir()
        self.socket_path = socket_path or os.path.join(run_dir, f"sortbots_drive_{os.getuid()}.sock")
        self.proc: subprocess.Popen | None = None
        self.log: deque[str] = deque(maxlen=LOG_LINES)
        self.lock = threading.Lock()

    # -- lifecycle -------------------------------------------------------------------------

    def running(self) -> bool:
        return self.proc is not None and self.proc.poll() is None

    def start(self) -> dict:
        with self.lock:
            if self.running():
                return {"ok": True, "already": True}
            if not Path(self.python).exists():
                return {"ok": False, "error": f"no lerobot env python at {self.python} "
                        "(set SORTBOTS_LEROBOT_PYTHON; docs/drive.md)"}
            argv = [self.python, str(BRIDGE)]
            if self.config:
                argv += ["--config", self.config]
            argv += ["serve", "--socket", self.socket_path]
            self.log.clear()
            # stdin stays a pipe we never write: the bridge exits on its EOF, so a serve.py
            # that dies by any means (even SIGKILL) takes the moving base down with it.
            # A clean env for conda's python: no ROS overlay may leak into it (CLAUDE.md).
            env = {k: v for k, v in os.environ.items()
                   if k not in ("AMENT_PREFIX_PATH", "PYTHONPATH", "LD_LIBRARY_PATH")}
            self.proc = subprocess.Popen(argv, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                         stderr=subprocess.STDOUT, env=env, cwd=str(REPO))
            threading.Thread(target=self._pump, args=(self.proc,), daemon=True).start()
        # The camera and the bus take a second or two to open; the socket appears first.
        for _ in range(100):
            if not self.running():
                return {"ok": False, "error": self._death_note()}
            if self._request({"op": "state"}, timeout=0.5) is not None:
                return {"ok": True}
            time.sleep(0.1)
        return {"ok": False, "error": "the bridge did not answer within 10 s"}

    def _pump(self, proc: subprocess.Popen) -> None:
        for raw in proc.stdout:
            self.log.append(raw.decode(errors="replace").rstrip())

    def _death_note(self) -> str:
        code = self.proc.returncode if self.proc else None
        tail = " | ".join(list(self.log)[-4:])
        return f"the bridge exited (code {code}){': ' + tail if tail else ''}"

    def stop(self) -> dict:
        with self.lock:
            proc = self.proc
            if proc is None or proc.poll() is not None:
                return {"ok": True}
            self._request({"op": "quit"}, timeout=1.0)
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.terminate()
                try:
                    proc.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    proc.kill()
            return {"ok": True}

    # -- requests ---------------------------------------------------------------------------

    def _connect(self, timeout: float) -> socket.socket:
        s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        s.settimeout(timeout)
        s.connect(self.socket_path)
        return s

    def _request(self, req: dict, timeout: float = 1.0) -> dict | None:
        try:
            with self._connect(timeout) as s:
                s.sendall(json.dumps(req).encode() + b"\n")
                line = s.makefile("rb").readline(1024 * 1024)
            return json.loads(line) if line else None
        except (OSError, ValueError):
            return None

    def state(self) -> dict:
        out = {"running": self.running(), "log": list(self.log)[-8:]}
        if not out["running"]:
            if self.proc is not None:
                out["error"] = self._death_note()
            return out
        st = self._request({"op": "state"})
        if st is None:
            out["error"] = "the bridge is running but not answering"
        else:
            out.update(st)
        return out

    def cmd(self, x, y, w) -> dict:
        if not self.running():
            return {"ok": False, "error": "not connected"}
        r = self._request({"op": "cmd", "x": x, "y": y, "w": w}, timeout=0.5)
        return r if r is not None else {"ok": False, "error": "the bridge did not answer"}

    def frame(self) -> bytes | None:
        if not self.running():
            return None
        try:
            with self._connect(1.0) as s:
                s.sendall(b'{"op": "frame"}\n')
                f = s.makefile("rb")
                head = json.loads(f.readline(4096) or b"{}")
                n = int(head.get("len") or 0)
                if n <= 0:
                    return None
                data = f.read(n)
            return data if len(data) == n else None
        except (OSError, ValueError):
            return None
