"""scripts/drive_bridge.py and webui/drive_proxy.py: the Drive view's wheel kinematics, the
command watchdog, and serve.py's gate on the drive routes. No hardware: the bridge runs
with its camera off and a serial port that does not exist, so nothing is ever opened."""
import json
import math
import os
import socket
import subprocess
import sys
import threading
import time
import urllib.error
import urllib.request
from pathlib import Path

import yaml

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "scripts"))
sys.path.insert(0, str(REPO / "webui"))
import drive_bridge as db  # noqa: E402
from drive_proxy import DriveProxy  # noqa: E402

CFG = db.load_config()


def _cfg(tmp_path, **over):
    cfg = {**CFG, "port": str(tmp_path / "no_such_tty"), "camera": {"enabled": False}, **over}
    p = tmp_path / "drive.yaml"
    p.write_text(yaml.safe_dump(cfg))
    return cfg, str(p)


# --------------------------------------------------------------------------- kinematics

def test_rotation_turns_every_wheel_the_same_way():
    raw = db.body_to_wheel_raw(0, 0, 1.0, CFG)
    assert raw[0] == raw[1] == raw[2] > 0


def test_forward_leaves_the_back_wheel_still_and_opposes_the_sides():
    left, back, right = db.body_to_wheel_raw(0.2, 0, 0, CFG)
    assert back == 0 and left == -right != 0


def test_strafe_left_is_mostly_the_back_wheel():
    left, back, right = db.body_to_wheel_raw(0, 0.2, 0, CFG)
    assert abs(back) > abs(left) and left == right


def test_matches_lerobot_lekiwi_where_it_is_installed():
    # The lerobot env has the original; system python3 (the test runner) usually doesn't.
    try:
        from lerobot.robots.lekiwi.lekiwi import LeKiwi
    except Exception:
        return
    for x, y, w in [(0.2, 0, 0), (0, 0.1, 0), (0.1, -0.2, 1.0), (0.3, 0.3, 3.0)]:
        # LeKiwi takes the turn in deg/s, the bridge in rad/s; it only touches static helpers.
        want = LeKiwi._body_to_wheel_raw(LeKiwi, x, y, math.degrees(w))
        assert db.body_to_wheel_raw(x, y, w, CFG) == [want["base_left_wheel"], want["base_back_wheel"], want["base_right_wheel"]]


def test_past_max_raw_all_wheels_scale_together():
    raw = db.body_to_wheel_raw(5.0, 0, 0, CFG)
    assert max(abs(r) for r in raw) <= CFG["max_raw"]
    slow = db.body_to_wheel_raw(0.1, 0, 0, CFG)
    assert raw[0] / raw[2] == slow[0] / slow[2]  # same direction of travel


def test_clamp_keeps_a_diagonal_no_faster_than_straight_and_drops_junk():
    x, y, w = db.clamp_cmd(1, 1, 99, 0.3, 1.0)
    assert math.isclose(math.hypot(x, y), 0.3) and x == y and w == 1.0
    assert db.clamp_cmd(float("nan"), "fast", None, 0.3, 1.0) == (0.0, 0.0, 0.0)
    assert db.clamp_cmd(float("inf"), 0, -5, 0.3, 1.0) == (0.0, 0.0, -1.0)


def test_limits_are_the_fastest_level():
    xy, w = db.limits(CFG)
    assert xy == CFG["speed_levels"][-1]["xy_mps"]
    assert math.isclose(w, math.radians(CFG["speed_levels"][-1]["theta_dps"]))


def test_ramp():
    assert db.ramp(0, 1, 0.1) == 0.1 and db.ramp(0.95, 1, 0.1) == 1 and db.ramp(0, -1, 0.1) == -0.1


# --------------------------------------------------------------------------- holders

def test_holders_names_the_process_with_the_device_open(tmp_path):
    dev = tmp_path / "ttyFAKE"
    dev.write_text("")
    with open(dev):
        child = subprocess.Popen([sys.executable, "-c", f"f=open({str(dev)!r}); import time; time.sleep(30)"])
        try:
            for _ in range(50):
                found = db.holders(str(dev))
                if found:
                    break
                time.sleep(0.05)
            assert [pid for pid, _ in found] == [child.pid]  # ourselves never counted
            assert "time.sleep" in found[0][1]
        finally:
            child.kill()
            child.wait()


# --------------------------------------------------------------------------- watchdog

class FakeBus:
    def __init__(self):
        self.writes = []

    def write(self, raw):
        self.writes.append((time.monotonic(), list(raw)))

    def volts(self):
        return 12.1


def test_wheels_stop_after_the_watchdog_and_never_jump():
    bridge = db.Bridge(CFG)
    fake = FakeBus()
    bridge.base.bus = True  # "open"; writes go to the fake
    bridge.base.write, bridge.base.volts = fake.write, fake.volts
    threading.Thread(target=bridge._control, daemon=True).start()
    try:
        t_end = time.monotonic() + 0.6
        while time.monotonic() < t_end:  # held for 0.6 s at 20 Hz, like the page
            bridge.handle({"op": "cmd", "x": 9, "y": 0, "w": 0}, None)
            time.sleep(0.05)
        moving = fake.writes[-1][1]
        time.sleep(CFG["watchdog_s"] + 0.6)  # released: watchdog, then the ramp down
    finally:
        bridge.done.set()
    assert moving != [0, 0, 0]
    assert fake.writes[-1][1] == [0, 0, 0]
    # clamped to the top level: never past its wheel speed
    top = db.body_to_wheel_raw(CFG["speed_levels"][-1]["xy_mps"], 0, 0, CFG)
    assert max(abs(r) for _, w in fake.writes for r in w) <= max(abs(r) for r in top)
    # ramped: no tick changes a wheel by more than one accel step
    step = db.body_to_wheel_raw(CFG["accel_mps2"] / db.CONTROL_HZ, 0, 0, CFG)
    for (_, a), (_, b) in zip(fake.writes, fake.writes[1:]):
        assert all(abs(p - q) <= abs(s) + 1 for p, q, s in zip(a, b, step))


def test_a_lost_bus_is_reopened_stopped_and_the_old_command_not_replayed():
    bridge = db.Bridge(CFG)
    fake, opens = FakeBus(), []
    state = {"fail": False}

    def write(raw):
        if state["fail"]:
            state["fail"] = False
            raise ConnectionError("port vanished")
        fake.write(raw)

    def reopen():
        opens.append(time.monotonic())
        bridge.base.bus = True
        fake.write([0, 0, 0])  # Base.open writes Goal_Velocity 0 before anything else

    bridge.base.bus = True
    bridge.base.write, bridge.base.volts, bridge.base.open = write, fake.volts, reopen
    threading.Thread(target=bridge._control, daemon=True).start()
    try:
        bridge.handle({"op": "cmd", "x": 0.3, "y": 0, "w": 0}, None)
        time.sleep(0.1)
        assert fake.writes[-1][1] != [0, 0, 0]  # moving
        state["fail"] = True
        time.sleep(0.3)  # the first retry is the very next tick
        assert len(opens) == 1 and bridge.base_error is None
        # From the reopen on: stopped, though the operator's last command asked for 0.3 m/s
        # and is still within the watchdog of when it was sent.
        assert all(w == [0, 0, 0] for t, w in fake.writes if t >= opens[0])
    finally:
        bridge.done.set()


# --------------------------------------------------------------------------- proxy + bridge process

def test_proxy_runs_the_bridge_without_hardware_and_reports_why(tmp_path):
    _, cfg_path = _cfg(tmp_path)
    proxy = DriveProxy(python=sys.executable, config=cfg_path, socket_path=str(tmp_path / "d.sock"))
    try:
        assert proxy.start()["ok"]
        st = proxy.state()
        assert st["running"] and not st["base"]["ok"] and "does not exist" in st["base"]["error"]
        assert not st["camera"]["ok"] and "camera off" in st["camera"]["error"]
        r = proxy.cmd(5, 0, 0)
        assert r["ok"] and r["target"][0] == CFG["speed_levels"][-1]["xy_mps"]
        assert proxy.frame() is None
    finally:
        proxy.stop()
    assert not proxy.running() and not (tmp_path / "d.sock").exists()


def test_bridge_exits_when_its_parent_goes(tmp_path):
    _, cfg_path = _cfg(tmp_path)
    sock = str(tmp_path / "d.sock")
    p = subprocess.Popen([sys.executable, str(REPO / "scripts" / "drive_bridge.py"), "--config", cfg_path,
                          "serve", "--socket", sock], stdin=subprocess.PIPE, stdout=subprocess.DEVNULL)
    try:
        for _ in range(100):
            if os.path.exists(sock):
                break
            time.sleep(0.05)
        p.stdin.close()  # serve.py gone
        assert p.wait(timeout=5) == 0
    finally:
        if p.poll() is None:
            p.kill()


# --------------------------------------------------------------------------- serve.py gate

def _free_port():
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def _serve(tmp_path, *extra):
    _, cfg_path = _cfg(tmp_path)
    port = _free_port()
    env = {**os.environ, "SORTBOTS_LEROBOT_PYTHON": sys.executable, "SORTBOTS_DRIVE_CONFIG": cfg_path,
           "SORTBOTS_SESSIONS_DIR": str(tmp_path / "sessions"),
           "XDG_RUNTIME_DIR": str(tmp_path)}
    p = subprocess.Popen([sys.executable, str(REPO / "webui" / "serve.py"), "--port", str(port),
                          "--host", "127.0.0.1", "--platform", "real", *extra],
                         env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    base = f"http://127.0.0.1:{port}"
    for _ in range(100):
        try:
            urllib.request.urlopen(base + "/api/version", timeout=1)
            return p, base
        except OSError:
            time.sleep(0.1)
    p.kill()
    raise RuntimeError("serve.py did not start")


def _get(url, headers=None):
    return json.loads(urllib.request.urlopen(urllib.request.Request(url, headers=headers or {}), timeout=5).read())


def _post(url, body, headers=None):
    req = urllib.request.Request(url, data=json.dumps(body).encode(), method="POST",
                                 headers={"Content-Type": "application/json", **(headers or {})})
    try:
        return 200, json.loads(urllib.request.urlopen(req, timeout=15).read())
    except urllib.error.HTTPError as e:
        return e.code, json.loads(e.read())


def test_serve_without_drive_offers_nothing(tmp_path):
    p, base = _serve(tmp_path)
    try:
        assert _get(base + "/api/drive")["enabled"] is False
        assert _post(base + "/api/drive/cmd", {"x": 0.1})[0] == 403
    finally:
        p.kill()


def test_serve_drive_refuses_proxied_requests_and_drives_local_ones(tmp_path):
    p, base = _serve(tmp_path, "--drive")
    try:
        for h in ({"X-Forwarded-For": "1.2.3.4"}, {"Tailscale-Funnel-Request": "?1"}, {"Forwarded": "for=1.2.3.4"}):
            assert _get(base + "/api/drive", h)["enabled"] is False
            assert _post(base + "/api/drive/start", {}, h)[0] == 403
            assert _post(base + "/api/drive/cmd", {"x": 0.1}, h)[0] == 403
        assert _get(base + "/api/drive")["enabled"] is True
        code, r = _post(base + "/api/drive/start", {})
        assert code == 200 and r["ok"], r
        st = _get(base + "/api/drive")
        assert st["running"] and "does not exist" in st["base"]["error"]
        code, r = _post(base + "/api/drive/cmd", {"x": 0.1, "y": 0, "w": 0})
        assert code == 200 and r["ok"]
        assert _post(base + "/api/drive/stop", {})[1]["ok"]
        assert _get(base + "/api/drive")["running"] is False
    finally:
        p.terminate()
        p.wait(timeout=10)
