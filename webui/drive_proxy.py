"""serve.py's half of the Drive view: starts scripts/drive_bridge.py and talks to it.

The bridge runs in the `lerobot` conda env (pyrealsense2, cv2, lerobot's servo bus) and
serve.py in the system python3, so they are two processes. They talk over a unix socket
(one JSON line a request), never a TCP port: serve.py stays the only HTTP server, and the
process that moves the robot is not reachable from the network by itself.

The bridge holds the servo bus and the D435 only while it runs, so it is started on the
operator's Connect and stopped on Disconnect, not with serve.py: the arm's own tools
(scripts/head.py, arm.sh pad) need the same board and camera the rest of the time.

When `handeye pick` is running (scripts/handeye.py, its page on 127.0.0.1:8765), IT owns the
camera, the arm, the magnet and the base, so the bridge can't run at all. The Drive view then goes
through handeye instead: its preview frame, its /base input for driving, and its one-button pick
(key `o`: find the package, pick it, carry it to the taught place pose, release, back to rest).

The operator can also START it from the Drive view (start_pick). handeye pick is a terminal
program (raw keyboard mode on stdin, a status line it rewrites), so it runs on a pseudo-terminal
whose master this process holds and drains; its output becomes the view's log. Stopping it is its
own quit (`x` twice: torque goes off), so the page asks the operator to support the arm first.

One Start for both (start_robot): the arm's board plugged in -> handeye pick; not -> the drive-only
bridge, same view. And if the arm's board goes away while the pick runs (unplugged, or dropped off
USB), the pick is quit and the bridge started in its place, so the base can still be driven.
"""
from __future__ import annotations

import glob
import json
import os
import pty
import re
import signal
import socket
import subprocess
import tempfile
import threading
import time
import urllib.error
import urllib.request
from collections import deque
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
BRIDGE = REPO / "scripts" / "drive_bridge.py"
DEFAULT_PYTHON = Path.home() / "miniconda3" / "envs" / "lerobot" / "bin" / "python"
LOG_LINES = 40
HANDEYE = os.environ.get("SORTBOTS_HANDEYE_URL", "http://127.0.0.1:8765")  # tests point it at a fake
CONFIG = REPO / "configs" / "drive_base.yaml"
# handeye's keys for the pad's buttons: `o` is its one-button pick-and-place; any key aborts a move
# in progress, and `c` is the one with no action of its own at rest (it only cancels a calibration look).
PICK_KEY, ABORT_KEY = "o", "c"
# How the Drive view starts handeye pick: arm.sh finds the arm's board by USB serial and refuses a
# ROS-sourced env. SORTBOTS_HANDEYE_CMD (a JSON argv) replaces it in the tests.
HANDEYE_ARGV = json.loads(os.environ["SORTBOTS_HANDEYE_CMD"]) if os.environ.get("SORTBOTS_HANDEYE_CMD") \
    else ["bash", str(REPO / "scripts" / "arm.sh"), "head", "handeye", "pick"]
# The arm+head board by its USB serial (scripts/arm.sh's ARM_BOARD_SERIAL): present = the arm is plugged in.
ARM_BOARD_GLOB = "/dev/serial/by-id/*" + os.environ.get("ARM_BOARD_SERIAL", "5B91029024") + "*"
ARM_GONE_CHECKS = 2  # the board missing this many 0.5 s checks in a row: a re-enumeration blip is shorter
ANSI = re.compile(r"\x1b\[[0-9;?]*[A-Za-z]")


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
        self.pick_proc: subprocess.Popen | None = None
        self.pick_log: deque[str] = deque(maxlen=LOG_LINES)
        self.stopping_pick = False  # the operator's Stop, not a crash: no fallback to driving
        self.note: str | None = None  # why the view changed mode by itself (the arm went away)

    # -- lifecycle -------------------------------------------------------------------------

    def running(self) -> bool:
        return self.proc is not None and self.proc.poll() is None

    def start(self) -> dict:
        if not self.running() and self.handeye_state() is not None:
            return {"ok": True, "backend": "handeye"}  # it owns the hardware; nothing to start
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

    # -- handeye pick ------------------------------------------------------------------------

    def _handeye(self, path: str, timeout: float = 0.5) -> bytes | None:
        try:
            with urllib.request.urlopen(HANDEYE + path, timeout=timeout) as r:
                return r.read()
        except (OSError, ValueError):
            return None

    def handeye_state(self) -> dict | None:
        """handeye pick's /state, or None if it isn't running (or is an older one without it)."""
        raw = self._handeye("/state")
        try:
            st = json.loads(raw) if raw else None
        except ValueError:
            return None
        return st if isinstance(st, dict) else None

    def _speed_levels(self) -> list:
        try:
            import yaml
            with open(self.config or CONFIG) as f:
                return yaml.safe_load(f).get("speed_levels") or []
        except (OSError, ValueError, AttributeError):
            return []

    @staticmethod
    def arm_board() -> bool:
        return bool(glob.glob(ARM_BOARD_GLOB))

    def start_robot(self) -> dict:
        """The view's one Start: the arm and its pick if the arm's board is plugged in, else the
        camera and the wheels only."""
        self.note = None
        if self.handeye_state() is not None or self.pick_running() or self.running():
            return {"ok": True, "already": True}
        if self.arm_board():
            return {**self.start_pick(), "mode": "arm"}
        r = self.start()
        if r.get("ok"):
            self.note = "The arm isn't plugged in: camera and wheels only."
        return {**r, "mode": "drive"}

    def stop_robot(self) -> dict:
        self.note = None
        if self.pick_running() or self.handeye_state() is not None:
            return self.stop_pick()
        return self.stop()

    def _watch_pick(self, proc: subprocess.Popen) -> None:
        """While our pick runs: the arm's board gone -> quit it. After it exits on its own (not the
        operator's Stop) with the arm gone -> the drive-only bridge, so the base still drives."""
        gone = 0
        while proc.poll() is None:
            gone = 0 if self.arm_board() else gone + 1
            if gone >= ARM_GONE_CHECKS and not self.stopping_pick:
                self.log.append("[drive] the arm's board went away: quitting the pick")
                self._quit_pick(proc)
                break
            time.sleep(0.5)
        proc.wait()
        if self.stopping_pick or self.arm_board():
            return  # stopped on purpose, or it crashed with the arm there: show why, don't hide it
        # The note first: the page must never see the view switch mode with no reason given.
        self.note = "The arm was unplugged: the pick stopped, camera and wheels only now."
        r = self.start()
        if not r.get("ok"):
            self.note = f"The arm was unplugged and driving did not start: {r.get('error')}"

    def pick_running(self) -> bool:
        return self.pick_proc is not None and self.pick_proc.poll() is None

    def start_pick(self) -> dict:
        """Start handeye pick on a pty. Returns at once: it takes a few seconds (camera, magnet board
        boot, the move to rest) before its page answers, and state() says `pick_starting` till then."""
        if self.handeye_state() is not None:
            return {"ok": True, "already": True}
        with self.lock:
            if self.pick_running():
                return {"ok": True, "starting": True}
        if self.running():
            self.stop()  # the drive-only bridge holds the camera and the base board
        with self.lock:
            master, slave = pty.openpty()
            env = {k: v for k, v in os.environ.items()
                   if k not in ("AMENT_PREFIX_PATH", "PYTHONPATH", "LD_LIBRARY_PATH")}
            env["TERM"] = "dumb"
            self.pick_log.clear()
            try:
                # its own session: a Ctrl-C meant for serve.py's terminal must not reach the arm
                self.pick_proc = subprocess.Popen(HANDEYE_ARGV, stdin=slave, stdout=slave, stderr=slave,
                                                  env=env, cwd=str(REPO), start_new_session=True, close_fds=True)
            finally:
                os.close(slave)
            threading.Thread(target=self._pump_pty, args=(master,), daemon=True).start()
            self.stopping_pick = False
            threading.Thread(target=self._watch_pick, args=(self.pick_proc,), daemon=True).start()
        time.sleep(0.5)  # arm.sh's own refusals (board unplugged, no permission) come back at once
        if not self.pick_running():
            return {"ok": False, "error": self._pick_death_note()}
        return {"ok": True, "starting": True}

    def _pump_pty(self, master: int) -> None:
        # Drained always: a full pty buffer would block handeye's status prints, and so its loop.
        buf = ""
        while True:
            try:
                data = os.read(master, 4096)
            except OSError:  # EIO: the child closed its side (exited)
                break
            if not data:
                break
            buf += ANSI.sub("", data.decode(errors="replace"))
            *lines, buf = re.split(r"[\r\n]+", buf)
            self.pick_log.extend(l.strip() for l in lines if l.strip())
        if buf.strip():
            self.pick_log.append(buf.strip())
        os.close(master)

    def _pick_death_note(self) -> str:
        code = self.pick_proc.returncode if self.pick_proc else None
        tail = " | ".join(list(self.pick_log)[-3:])
        return f"handeye pick exited (code {code}){': ' + tail if tail else ''}"

    def stop_pick(self) -> dict:
        """handeye's own quit (`x` twice: magnet off, base stopped, torque off), then signals if it
        doesn't go. Only a pick this process started is signalled; one from a terminal gets the keys."""
        self.stopping_pick = True
        proc = self.pick_proc
        if proc is None or proc.poll() is not None:
            return {"ok": self._quit_keys() or proc is not None}
        return {"ok": self._quit_pick(proc)}

    def _quit_keys(self) -> bool:
        answered = self._handeye("/cmd?k=x") is not None
        if answered:
            time.sleep(0.3)
            self._handeye("/cmd?k=x")
        return answered

    def _quit_pick(self, proc: subprocess.Popen) -> bool:
        self._quit_keys()
        for sig, wait in ((None, 8.0), (signal.SIGINT, 5.0), (signal.SIGKILL, 3.0)):
            if sig is not None:
                try:
                    proc.send_signal(sig)
                except ProcessLookupError:
                    break
            try:
                proc.wait(timeout=wait)
                break
            except subprocess.TimeoutExpired:
                continue
        return proc.poll() is not None

    def pick(self) -> dict:
        if self.running():
            return {"ok": False, "error": "pick needs `handeye pick` running, not the drive-only bridge: "
                    "Disconnect, then start it (scripts/arm.sh head handeye pick)"}
        if self._handeye(f"/cmd?k={PICK_KEY}") is None:
            return {"ok": False, "error": "`handeye pick` is not running (scripts/arm.sh head handeye pick)"}
        return {"ok": True}

    def abort(self) -> dict:
        return {"ok": self._handeye(f"/cmd?k={ABORT_KEY}") is not None}

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
        return {**self._state(), "arm_board": self.arm_board(), "note": self.note}

    def _state(self) -> dict:
        out = {"running": self.running(), "log": list(self.log)[-8:]}
        if not out["running"]:
            hs = self.handeye_state()
            if hs is None and self.pick_running():
                return {"running": False, "pick_starting": True, "pick_log": list(self.pick_log)[-4:],
                        "speed_levels": self._speed_levels()}
            if hs is None and self.pick_proc is not None:
                out["pick_error"] = self._pick_death_note()
            if hs is not None:
                return {"running": True, "backend": "handeye", "pick": hs, "pick_owned": self.pick_running(),
                        "camera": {"ok": True, "error": None, "fps": None},
                        "base": {"ok": bool(hs.get("base")),
                                 "error": None if hs.get("base") else "handeye pick has no base (base board unplugged, "
                                 "or started with --base-back-cm 0)"},
                        "speed_levels": self._speed_levels()}
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
            try:
                q = "&".join(f"{k}={float(v):.4f}" for k, v in (("x", x), ("y", y), ("w", w)))
            except (TypeError, ValueError):
                q = "x=0&y=0&w=0"
            if self._handeye("/base?" + q) is not None:
                return {"ok": True, "backend": "handeye"}
            return {"ok": False, "error": "not connected"}
        r = self._request({"op": "cmd", "x": x, "y": y, "w": w}, timeout=0.5)
        return r if r is not None else {"ok": False, "error": "the bridge did not answer"}

    def frame(self) -> bytes | None:
        if not self.running():
            return self._handeye("/handeye_view.jpg", timeout=1.0)
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
