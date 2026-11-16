#!/usr/bin/env python3
"""The real robot's omni base and D435 colour stream, for the operator view's Drive view.

Runs in the `lerobot` conda env (pyrealsense2, cv2, lerobot's servo bus), NEVER in a ROS
shell. Nothing here is a ROS node: on a laptop with no ROS, the operator view still gets a
live camera and drives the wheels through serve.py.

    drive_bridge.py serve --socket PATH   started by serve.py --drive (webui/drive_proxy.py)
    drive_bridge.py scan                  who answers on the wheel bus (ids, volts, temp)
    drive_bridge.py setup left|back|right give ONE new wheel servo its id; it must be ALONE
    drive_bridge.py wheels                spin each wheel slowly in turn, to check ids and signs

`serve` speaks one JSON line per request on a unix socket, never a TCP port: serve.py is the
only HTTP server (CLAUDE.md "One GUI"), and this process must not be reachable from the
network on its own. Requests: {"op": "state"}, {"op": "cmd", "x", "y", "w"} (m/s, m/s,
rad/s, body frame: x forward, y left, w counter-clockwise), {"op": "frame"} (answered by a
JSON header line, then that many JPEG bytes), {"op": "quit"}.

Safety, in the order it bites:
- The wheels stop WATCHDOG_S after the last command: the page sends commands only while
  the pad's deadman is held, so a closed tab, a dead link or a crashed browser stops the robot.
- serve.py holds this process's stdin. EOF on it (serve.py gone, however it died) stops the
  wheels and exits, so the bridge is never orphaned holding a moving base.
- One process owns a servo bus and the D435 at a time. Both are checked BEFORE opening, and
  a holder is named (pid + command) instead of opened under it: `handeye pick` opens the
  same camera and the wheels' board (it backs the base off the table to place).
- Only the wheel ids are ever written, so any other servo on the board is left as it was.
"""
import os

# One BLAS thread before numpy loads (cv2 pulls it in): the default thread-per-core OpenBLAS
# pool starved head.py's 50 Hz loop on this laptop (2026-11-15), and the same applies here.
for _v in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS"):
    os.environ.setdefault(_v, "1")

import argparse
import json
import math
import signal
import socket
import sys
import threading
import time
from pathlib import Path

# yaml, lerobot, cv2 and pyrealsense2 are imported where they are used, so the pure helpers
# below load under the system python3 that runs tests/drive_bridge_test.py.

REPO = Path(__file__).resolve().parents[1]
CONFIG = REPO / "configs" / "drive_base.yaml"
WHEELS = ("base_left_wheel", "base_back_wheel", "base_right_wheel")
STEPS_PER_DEG = 4096.0 / 360.0
CONTROL_HZ = 50
# Goal_Velocity is re-sent at least this often even when unchanged: a lost sync write (no
# servo answers one) must not leave a wheel on an old speed for long.
RESEND_S = 0.2


# ----------------------------------------------------------------------------- pure helpers

def load_config(path=CONFIG):
    import yaml
    with open(path) as f:
        cfg = yaml.safe_load(f)
    missing = [w for w in WHEELS if w not in (cfg.get("wheels") or {})]
    if missing:
        raise ValueError(f"{path}: wheels: missing {missing}")
    return cfg


def limits(cfg):
    """(max m/s, max rad/s): the fastest speed level. Every command is clamped to it."""
    top = cfg["speed_levels"][-1]
    return float(top["xy_mps"]), math.radians(float(top["theta_dps"]))


def clamp_cmd(x, y, w, max_xy, max_w):
    """Clamp a body command. x/y as a vector (a diagonal is no faster than straight), w alone.
    Anything that is not a finite number becomes 0: this is the last stop before the motors."""
    vals = []
    for v in (x, y, w):
        try:
            v = float(v)
        except (TypeError, ValueError):
            v = 0.0
        vals.append(v if math.isfinite(v) else 0.0)
    x, y, w = vals
    n = math.hypot(x, y)
    if n > max_xy > 0:
        x, y = x * max_xy / n, y * max_xy / n
    elif max_xy <= 0:
        x = y = 0.0
    w = max(-max_w, min(max_w, w))
    return x, y, w


def ramp(cur, target, max_step):
    """Move cur toward target by at most max_step."""
    d = target - cur
    return target if abs(d) <= max_step else cur + math.copysign(max_step, d)


def degps_to_raw(degps):
    # Goal_Velocity is sign-magnitude with the sign on bit 15: at most 32767 either way.
    return max(-0x7FFF, min(0x7FFF, int(round(degps * STEPS_PER_DEG))))


def body_to_wheel_raw(x, y, w, cfg):
    """Body velocity (m/s, m/s, rad/s) -> raw Goal_Velocity per wheel, left/back/right.

    lerobot's LeKiwi._body_to_wheel_raw without numpy: each wheel's rim speed is the body
    velocity projected on its rolling direction plus base_radius * w; past max_raw on any
    wheel, all three scale down together so the robot still goes where it was told.
    """
    r_wheel, r_base = float(cfg["wheel_radius_m"]), float(cfg["base_radius_m"])
    degps = []
    for a in cfg["wheel_angles_deg"]:
        a = math.radians(a - 90)
        rim = math.cos(a) * x + math.sin(a) * y + r_base * w
        degps.append(math.degrees(rim / r_wheel))
    peak = max(abs(d) for d in degps) * STEPS_PER_DEG
    if peak > cfg["max_raw"]:
        degps = [d * cfg["max_raw"] / peak for d in degps]
    return [degps_to_raw(d) for d in degps]


def holders(dev, proc="/proc"):
    """[(pid, command)] of OTHER processes with `dev` open. Linux /proc only; a process we may
    not inspect (another user's) is skipped, which only means it isn't named."""
    try:
        target = os.path.realpath(dev)
    except OSError:
        return []
    me, found = os.getpid(), []
    for pid in os.listdir(proc):
        if not pid.isdigit() or int(pid) == me:
            continue
        fd_dir = os.path.join(proc, pid, "fd")
        try:
            fds = os.listdir(fd_dir)
        except OSError:
            continue
        for fd in fds:
            try:
                if os.path.realpath(os.path.join(fd_dir, fd)) == target:
                    break
            except OSError:
                continue
        else:
            continue
        try:
            with open(os.path.join(proc, pid, "cmdline"), "rb") as f:
                cmd = f.read().replace(b"\0", b" ").decode(errors="replace").strip()
        except OSError:
            cmd = "?"
        found.append((int(pid), cmd))
    return found


def realsense_video_nodes(sysfs="/sys/class/video4linux"):
    """The /dev/videoN nodes the D435 shows up as (librealsense opens them through V4L)."""
    nodes = []
    try:
        entries = sorted(os.listdir(sysfs))
    except OSError:
        return nodes
    for n in entries:
        try:
            with open(os.path.join(sysfs, n, "name")) as f:
                if "realsense" in f.read().lower():
                    nodes.append(f"/dev/{n}")
        except OSError:
            continue
    return nodes


def busy_message(what, held):
    pid, cmd = held[0]
    return (f"{what} is in use by pid {pid} ({cmd[:120]}). Only one process can own it: "
            f"stop that one first, then Connect again.")


# ----------------------------------------------------------------------------- hardware

class Base:
    """The three wheel servos, in velocity mode, on their own board (configs/drive_base.yaml)."""

    def __init__(self, cfg):
        self.cfg = cfg
        self.port = cfg["port"]
        self.ids = {w: int(cfg["wheels"][w]) for w in WHEELS}
        self.bus = None

    def open(self):
        held = holders(self.port)
        if held:
            raise RuntimeError(busy_message(f"The servo bus {self.port}", held))
        if not os.path.exists(self.port):
            raise RuntimeError(f"{self.port} does not exist: is the servo board plugged in?")
        from lerobot.motors import Motor, MotorNormMode, SerialMotorsBus
        bus = SerialMotorsBus(port=self.port, motors={
            w: Motor(i, "sts3215", MotorNormMode.RANGE_M100_100) for w, i in self.ids.items()})
        try:
            bus.connect(handshake=True)  # pings every wheel id; names the missing ones
        except Exception as e:
            raise RuntimeError(f"wheel servos {self.ids} on {self.port}: {e}. "
                               f"`scripts/drive_bridge.py scan` lists what answers.") from e
        try:
            # Operating_Mode is EEPROM: torque off opens the lock, torque on closes it again
            # (lekiwi.py configure()). Wheels only: the arm's torque is left alone.
            bus.disable_torque(WHEELS)
            bus.set_operating_mode("velocity", WHEELS)
            bus.sync_write("Goal_Velocity", dict.fromkeys(WHEELS, 0), normalize=False)
            bus.enable_torque(WHEELS)
        except Exception:
            bus.disconnect(disable_torque=False)
            raise
        self.bus = bus

    def write(self, raw):
        self.bus.sync_write("Goal_Velocity", dict(zip(WHEELS, raw)), normalize=False)

    def volts(self):
        return self.bus.read("Present_Voltage", WHEELS[0], normalize=False, num_retry=1) / 10.0

    def close(self):
        if self.bus is None:
            return
        bus, self.bus = self.bus, None
        try:
            bus.sync_write("Goal_Velocity", dict.fromkeys(WHEELS, 0), normalize=False, num_retry=5)
            bus.disable_torque(WHEELS, num_retry=3)  # wheels free-roll, robot can be pushed
        finally:
            bus.disconnect(disable_torque=False)  # never the arm's torque


class Camera:
    """The D435's colour stream, kept as the latest JPEG."""

    def __init__(self, cfg):
        self.cfg = cfg.get("camera") or {}
        self.pipe = None
        self.jpeg = None
        self.seq = 0
        self.size = None
        self.fps = 0.0
        self.error = None
        self._stop = threading.Event()
        self._thread = None

    def open(self):
        if self.cfg.get("enabled") is False:
            raise RuntimeError("camera off in the config (camera: enabled: false)")
        for node in realsense_video_nodes():
            held = holders(node)
            if held:
                raise RuntimeError(busy_message("The D435", held))
        import pyrealsense2 as rs
        if not rs.context().query_devices().size():
            raise RuntimeError("no RealSense camera found: is the D435 plugged in (USB 3 port)?")
        w, h = int(self.cfg.get("width", 640)), int(self.cfg.get("height", 480))
        last = None
        for fps in self.cfg.get("fps", [30, 15]):
            pipe, conf = rs.pipeline(), rs.config()
            conf.enable_stream(rs.stream.color, w, h, rs.format.bgr8, int(fps))
            try:
                pipe.start(conf)
            except RuntimeError as e:
                last = e
                continue
            self.pipe, self.size = pipe, (w, h, int(fps))
            break
        if self.pipe is None:
            raise RuntimeError(f"D435 colour {w}x{h} would not start: {last}")
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self):
        import cv2
        import numpy as np
        q = [int(cv2.IMWRITE_JPEG_QUALITY), int(self.cfg.get("jpeg_quality", 70))]
        t0, n = time.monotonic(), 0
        while not self._stop.is_set():
            try:
                frames = self.pipe.wait_for_frames(2000)
            except RuntimeError as e:
                self.error = f"no frames from the D435 for 2 s: {e}"
                continue
            color = frames.get_color_frame()
            if not color:
                continue
            ok, buf = cv2.imencode(".jpg", np.asanyarray(color.get_data()), q)
            if not ok:
                continue
            self.jpeg, self.error = buf.tobytes(), None
            self.seq += 1
            n += 1
            if time.monotonic() - t0 >= 1.0:
                self.fps, t0, n = n / (time.monotonic() - t0), time.monotonic(), 0

    def close(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=3)
        if self.pipe:
            try:
                self.pipe.stop()
            except RuntimeError:
                pass
            self.pipe = None


# ----------------------------------------------------------------------------- serve

class Bridge:
    def __init__(self, cfg):
        self.cfg = cfg
        self.max_xy, self.max_w = limits(cfg)
        self.watchdog = float(cfg.get("watchdog_s", 0.3))
        self.lock = threading.Lock()
        self.target = (0.0, 0.0, 0.0)
        self.target_at = 0.0
        self.vel = (0.0, 0.0, 0.0)
        self.raw = [0, 0, 0]
        self.volts = None
        self.base_error = self.cam_error = None
        self.base_lost = False  # opened once, then lost: retried. Never opened: left to Connect
        self.base = Base(cfg)
        self.cam = Camera(cfg)
        self.done = threading.Event()

    def start(self):
        # Each half on its own: a camera with no wheels yet (ids not set up) is still useful.
        try:
            self.cam.open()
        except Exception as e:
            self.cam_error = str(e)
        try:
            self.base.open()
        except Exception as e:
            self.base_error = str(e)
        threading.Thread(target=self._control, daemon=True).start()

    def _control(self):
        dt = 1.0 / CONTROL_HZ
        dv = float(self.cfg.get("accel_mps2", 1.0)) * dt
        dw = math.radians(float(self.cfg.get("alpha_dps2", 180))) * dt
        last_raw, last_sent, last_volts, last_retry = None, 0.0, 0.0, 0.0
        while not self.done.is_set():
            now = time.monotonic()
            # A board that dropped off USB mid-drive leaves the servos on their last speed:
            # an STS3215 has no command timeout of its own. Reopening writes 0 before
            # anything else (Base.open), so keep trying while the fault lasts.
            if self.base.bus is None and self.base_lost and now - last_retry >= 0.5:
                last_retry = now
                try:
                    self.base.open()
                    self.base_error, self.base_lost, last_raw = None, False, None
                    with self.lock:
                        self.target_at = 0.0  # a command from before the fault is not replayed
                    self.vel = (0.0, 0.0, 0.0)
                except Exception:
                    pass
            with self.lock:
                target = self.target if now - self.target_at <= self.watchdog else (0.0, 0.0, 0.0)
            x, y, w = self.vel
            self.vel = (ramp(x, target[0], dv), ramp(y, target[1], dv), ramp(w, target[2], dw))
            raw = body_to_wheel_raw(*self.vel, self.cfg)
            self.raw = raw
            if self.base.bus is not None:
                try:
                    if raw != last_raw or now - last_sent >= RESEND_S:
                        self.base.write(raw)
                        last_raw, last_sent = raw, now
                    if now - last_volts >= 1.0:
                        self.volts, last_volts = self.base.volts(), now
                except Exception as e:  # board dropped off USB (docs/arm.md): say so, keep the camera
                    self.base_error = (f"lost the wheel bus ({e}); reconnecting. If the wheels are "
                                       "still turning, switch the base's power off")
                    try:
                        self.base.bus.disconnect(disable_torque=False)
                    except Exception:
                        pass
                    self.base.bus, self.base_lost = None, True
            time.sleep(max(0.0, dt - (time.monotonic() - now)))

    def state(self):
        return {
            "camera": {"ok": self.cam.pipe is not None and self.cam.jpeg is not None,
                       "error": self.cam_error or self.cam.error, "fps": round(self.cam.fps, 1),
                       "size": self.cam.size, "seq": self.cam.seq},
            "base": {"ok": self.base.bus is not None, "error": self.base_error,
                     "port": self.base.port, "ids": self.base.ids, "volts": self.volts,
                     "vel": [round(v, 3) for v in self.vel], "raw": self.raw},
            "speed_levels": self.cfg["speed_levels"],
            "watchdog_s": self.watchdog,
        }

    def handle(self, req, conn):
        op = req.get("op")
        if op == "cmd":
            with self.lock:
                self.target = clamp_cmd(req.get("x"), req.get("y"), req.get("w"), self.max_xy, self.max_w)
                self.target_at = time.monotonic()
            return {"ok": True, "target": self.target, "base": self.base.bus is not None}
        if op == "state":
            return self.state()
        if op == "frame":
            jpeg, seq = self.cam.jpeg, self.cam.seq
            if jpeg is None:
                return {"len": 0, "error": self.cam_error or self.cam.error or "no frame yet"}
            conn.sendall(json.dumps({"len": len(jpeg), "seq": seq}).encode() + b"\n" + jpeg)
            return None
        if op == "quit":
            self.done.set()
            return {"ok": True}
        return {"error": f"unknown op {op!r}"}

    def close(self):
        self.done.set()
        try:
            self.base.close()
        finally:
            self.cam.close()


def _serve_conn(bridge, conn):
    with conn:
        conn.settimeout(2.0)
        f = conn.makefile("rb")
        line = f.readline(64 * 1024)
        try:
            req = json.loads(line or b"{}")
        except ValueError:
            req = {}
        try:
            resp = bridge.handle(req if isinstance(req, dict) else {}, conn)
        except Exception as e:
            resp = {"error": str(e)}
        if resp is not None:
            conn.sendall(json.dumps(resp).encode() + b"\n")


def cmd_serve(a):
    cfg = load_config(a.config)
    try:
        os.unlink(a.socket)
    except FileNotFoundError:
        pass
    srv = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    old = os.umask(0o177)  # this user only: the socket moves a robot
    try:
        srv.bind(a.socket)
    finally:
        os.umask(old)
    srv.listen(16)
    bridge = Bridge(cfg)
    bridge.start()

    def watch_parent():
        # serve.py owns our stdin; EOF means it is gone, however it died.
        try:
            while sys.stdin.buffer.read(1):
                pass
        except (OSError, ValueError):
            pass
        bridge.done.set()
    if not a.no_parent:
        threading.Thread(target=watch_parent, daemon=True).start()
    signal.signal(signal.SIGTERM, lambda *_: bridge.done.set())
    signal.signal(signal.SIGINT, lambda *_: bridge.done.set())

    def accept():
        while not bridge.done.is_set():
            try:
                conn, _ = srv.accept()
            except OSError:
                return
            threading.Thread(target=_serve_conn, args=(bridge, conn), daemon=True).start()
    threading.Thread(target=accept, daemon=True).start()
    st = bridge.state()
    print(f"[drive_bridge] camera: {'ok' if st['camera']['ok'] or not st['camera']['error'] else st['camera']['error']}", flush=True)
    print(f"[drive_bridge] wheels: {'ok' if st['base']['ok'] else st['base']['error']}", flush=True)
    bridge.done.wait()
    print("[drive_bridge] stopping: wheels to 0, torque off", flush=True)
    srv.close()
    bridge.close()
    try:
        os.unlink(a.socket)
    except FileNotFoundError:
        pass


# ----------------------------------------------------------------------------- bench tools

def _open_scan_bus(cfg):
    from lerobot.motors import Motor, MotorNormMode, SerialMotorsBus
    held = holders(cfg["port"])
    if held:
        raise SystemExit(busy_message(cfg["port"], held))
    bus = SerialMotorsBus(port=cfg["port"], motors={
        w: Motor(int(i), "sts3215", MotorNormMode.RANGE_M100_100) for w, i in cfg["wheels"].items()})
    bus.connect(handshake=False)
    return bus


def cmd_scan(a):
    cfg = load_config(a.config)
    bus = _open_scan_bus(cfg)
    names = {int(i): w for w, i in cfg["wheels"].items()}
    try:
        found = bus._scan(bus._definition(WHEELS[0]))
        if not found:
            print("nothing answers at 1 Mbaud: 12 V supply on? servo cable seated?")
        for i in sorted(found):
            print(f"id {i:3d}  model {found[i]}  {names.get(i, '(not a wheel id in configs/drive_base.yaml)')}")
        missing = [f"{w}({i})" for i, w in sorted(names.items()) if i not in found]
        if missing:
            print("wheels missing:", " ".join(missing),
                  "\n  a NEW servo answers as id 1 (same as the arm's shoulder_pan): give it its id "
                  "with `drive_bridge.py setup left|back|right`, ALONE on the board")
    finally:
        bus.disconnect(disable_torque=False)


def cmd_setup(a):
    cfg = load_config(a.config)
    motor = f"base_{a.which}_wheel"
    bus = _open_scan_bus(cfg)
    print(f"Plug ONLY the {motor} servo into the board (unplug the arm, head and the other wheels),\n"
          f"12 V on. It will get id {cfg['wheels'][motor]} at 1 Mbaud.")
    input("press ENTER when it is the only servo on the bus...")
    # lerobot's setup_motor renumbers the FIRST servo a scan finds and does not check it is
    # alone: a forgotten arm cable would renumber shoulder_pan (head.py setup, same check).
    definition = bus._definition(motor)
    seen = []
    try:
        for rate in definition.baudrates:
            bus.set_baudrate(rate)
            seen += [(rate, i) for i in bus._scan(definition)]
    finally:
        bus.set_baudrate(bus.default_baudrate)
    if len(seen) != 1:
        bus.disconnect(disable_torque=False)
        raise SystemExit(f"need exactly one servo on the bus, found {len(seen)}: "
                         + (", ".join(f"id {i} @ {r}" for r, i in seen) or "none (12 V on?)"))
    rate, found_id = seen[0]
    print(f"found id {found_id} @ {rate} baud")
    bus.setup_motor(motor, initial_baudrate=rate, initial_id=found_id)
    bus.disconnect(disable_torque=False)
    print(f"{motor}: id {cfg['wheels'][motor]} set. Label it, then the next one.")


def cmd_wheels(a):
    """Each wheel alone at a slow speed for 1.5 s: the robot must be on blocks or held."""
    cfg = load_config(a.config)
    base = Base(cfg)
    input("Robot ON BLOCKS (wheels off the ground)? ENTER to spin each wheel in turn...")
    base.open()
    try:
        for k, w in enumerate(WHEELS):
            raw = [0, 0, 0]
            raw[k] = degps_to_raw(60)
            print(f"{w} (id {base.ids[w]}): +60 deg/s")
            t = time.monotonic()
            while time.monotonic() - t < 1.5:
                base.write(raw)
                time.sleep(0.1)
            base.write([0, 0, 0])
            time.sleep(0.5)
        # Pure rotation drives all three rims the same way (body_to_wheel_raw: r_base * w on
        # each), so this is the check that the three signs agree with each other.
        print("all three: +w (the robot should turn counter-clockwise seen from above)")
        raw = body_to_wheel_raw(0.0, 0.0, math.radians(30), cfg)
        t = time.monotonic()
        while time.monotonic() - t < 1.5:
            base.write(raw)
            time.sleep(0.1)
    finally:
        base.close()
    print("A wheel that went the other way from the other two is mounted or numbered differently\n"
          "from XLeRobot's: check its id against the left/back/right order in configs/drive_base.yaml.")


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--config", default=str(CONFIG))
    sub = p.add_subparsers(dest="cmd", required=True)
    s = sub.add_parser("serve")
    s.add_argument("--socket", required=True)
    s.add_argument("--no-parent", action="store_true",
                   help="don't exit on stdin EOF (running it by hand, not from serve.py)")
    sub.add_parser("scan")
    s = sub.add_parser("setup")
    s.add_argument("which", choices=["left", "back", "right"])
    sub.add_parser("wheels")
    a = p.parse_args()
    {"serve": cmd_serve, "scan": cmd_scan, "setup": cmd_setup, "wheels": cmd_wheels}[a.cmd](a)


if __name__ == "__main__":
    main()
