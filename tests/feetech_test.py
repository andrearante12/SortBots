"""nodes/feetech.py against the exact frames lerobot's SerialMotorsBus sends for the same
operations (captured on a pty, 2026-11-16), plus a fake servo on a pty for the request/reply
path. No hardware."""
import os
import pty
import sys
import threading
import tty
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "nodes"))
import feetech as ft  # noqa: E402

H = bytes.fromhex


def test_sync_velocity_matches_lerobot():
    assert ft.sync_velocity_frame({7: 100, 8: -100, 9: 0}) == H("ff ff fe 0d 83 2e 02 07 64 00 08 64 80 09 00 00 e1")
    assert ft.sync_velocity_frame({7: 3000, 8: -3000, 9: 32767}) == H("ff ff fe 0d 83 2e 02 07 b8 0b 08 b8 8b 09 ff 7f a5")


def test_writes_and_reads_match_lerobot():
    assert ft.write_frame(7, ft.TORQUE_ENABLE, 0) == H("ff ff 07 04 03 28 00 c9")
    assert ft.write_frame(9, ft.TORQUE_ENABLE, 1) == H("ff ff 09 04 03 28 01 c6")
    assert ft.write_frame(7, ft.OPERATING_MODE, 1) == H("ff ff 07 04 03 21 01 cf")
    assert ft.read_frame(7, ft.PRESENT_VOLTAGE) == H("ff ff 07 04 02 3e 01 b3")


def test_sign_magnitude_round_trip():
    for v in (0, 1, -1, 3000, -3000, 32767, -32767):
        assert ft.decode(ft.velocity(v), sign_bit=15) == v
    with pytest.raises(ValueError):
        ft.velocity(32768)


def test_parse_status_skips_noise_and_bad_checksum():
    good = ft.frame(7, 0x00, b"\x09\x03")  # status: error 0, model 777 (0x0309)
    bad = bytearray(good)
    bad[-1] ^= 0xFF
    assert ft.parse_status(b"\x00\xff" + bytes(bad) + good, 7) == (0, b"\x09\x03")
    assert ft.parse_status(good, 8) is None
    assert ft.parse_status(good[:-1], 7) is None


class FakeServos:
    """Answers reads/writes on a pty like STS3215s with the given ids would."""

    def __init__(self, ids, mode=1):
        self.master, slave = pty.openpty()
        tty.setraw(slave)
        self.path = os.ttyname(slave)
        self._slave = slave
        self.regs = {sid: {ft.MODEL_NUMBER[0]: 777, ft.OPERATING_MODE[0]: mode,
                           ft.TORQUE_ENABLE[0]: 0, ft.GOAL_VELOCITY[0]: 0} for sid in ids}
        self.writes, self.syncs = [], []
        self._stop = False
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        buf = b""
        while not self._stop:
            try:
                buf += os.read(self.master, 256)
            except OSError:
                return
            while True:
                i = buf.find(b"\xff\xff")
                if i < 0 or len(buf) < i + 4 or len(buf) < i + 4 + buf[i + 3]:
                    break
                n = buf[i + 3]
                sid, instr, params = buf[i + 2], buf[i + 4], buf[i + 5:i + 3 + n]
                buf = buf[i + 4 + n:]
                if instr == ft.SYNC_WRITE:
                    addr, size = params[0], params[1]
                    for k in range(2, len(params), size + 1):
                        tid = params[k]
                        val = ft.decode(params[k + 1:k + 1 + size], sign_bit=15)
                        self.syncs.append((tid, val))
                        if tid in self.regs:
                            self.regs[tid][addr] = val
                elif sid in self.regs and instr == ft.READ:
                    addr, size = params[0], params[1]
                    os.write(self.master, ft.frame(sid, 0, ft.encode(self.regs[sid][addr], size)))
                elif sid in self.regs and instr == ft.WRITE:
                    addr = params[0]
                    self.regs[sid][addr] = int.from_bytes(params[1:], "little")
                    self.writes.append((sid, addr, self.regs[sid][addr]))
                    os.write(self.master, ft.frame(sid, 0))

    def close(self):
        self._stop = True
        os.close(self.master)
        os.close(self._slave)


def _pty_baud_ok(path):
    # Some kernels refuse 1 Mbaud on a pty; the protocol logic is what's under test.
    try:
        b = ft.Bus(path)
        b.open()
        b.close()
        return True
    except Exception:
        return False


@pytest.fixture
def servos():
    s = FakeServos([7, 8, 9], mode=0)
    if not _pty_baud_ok(s.path):
        s.close()
        pytest.skip("pty refuses 1 Mbaud here")
    s.timeout = 0.2
    yield s
    s.close()


def test_open_sets_velocity_mode_once_and_torque_on(servos, monkeypatch):
    monkeypatch.setattr(ft.Bus, "__init__", lambda self, port, timeout_s=0.2: (setattr(self, "port", port), setattr(self, "timeout_s", 0.2), setattr(self, "fd", None))[-1])
    bus = ft.open_wheels(servos.path, [7, 8, 9])
    for sid in (7, 8, 9):
        assert servos.regs[sid][ft.OPERATING_MODE[0]] == 1
        assert servos.regs[sid][ft.TORQUE_ENABLE[0]] == 1
    mode_writes = [w for w in servos.writes if w[1] == ft.OPERATING_MODE[0]]
    assert len(mode_writes) == 3
    # torque was off before each mode write (the EEPROM lock)
    for sid in (7, 8, 9):
        seq = [w for w in servos.writes if w[0] == sid]
        assert seq.index((sid, ft.TORQUE_ENABLE[0], 0)) < seq.index((sid, ft.OPERATING_MODE[0], 1))
    bus.sync_velocity({7: 500, 8: -500, 9: 0})
    ft.release_wheels(bus, [7, 8, 9])
    for sid in (7, 8, 9):
        assert servos.regs[sid][ft.GOAL_VELOCITY[0]] == 0
        assert servos.regs[sid][ft.TORQUE_ENABLE[0]] == 0

    # Already in velocity mode: no EEPROM write the second time.
    n = len([w for w in servos.writes if w[1] == ft.OPERATING_MODE[0]])
    ft.release_wheels(ft.open_wheels(servos.path, [7, 8, 9]), [7, 8, 9])
    assert len([w for w in servos.writes if w[1] == ft.OPERATING_MODE[0]]) == n


def test_missing_servo_is_named(servos, monkeypatch):
    monkeypatch.setattr(ft.Bus, "__init__", lambda self, port, timeout_s=0.05: (setattr(self, "port", port), setattr(self, "timeout_s", 0.05), setattr(self, "fd", None))[-1])
    with pytest.raises(ft.BusError, match="servo 10"):
        ft.open_wheels(servos.path, [7, 10])
