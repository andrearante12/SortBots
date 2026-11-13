"""scripts/handeye.py MagnetLink: the page shows the magnet as the BOARD reports it."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
import handeye  # noqa: E402


class FakeSerial:
    def __init__(self):
        self.lines = []

    @property
    def in_waiting(self):
        return len(self.lines)

    def readline(self):
        return (self.lines.pop(0) + "\n").encode()


class FakeBoard:
    """Replies like hardware/magnet_esp32/main.py; `reset_on_on` mimics the coil's inrush
    resetting the ESP32 (it then prints 'ok ready' and the coil is off)."""

    def __init__(self, reset_on_on=False, dead=False):
        self.s, self.coil, self.reset_on_on, self.dead, self.closed = FakeSerial(), False, reset_on_on, dead, False

    def _check(self):
        if self.dead:
            raise SystemExit("magnet: no reply")

    def on(self, seconds=None):
        self._check()
        if self.reset_on_on:
            self.coil = False
            self.s.lines.append("ok ready")
            return "ok on"
        self.coil = True
        return "ok on"

    def off(self):
        self._check()
        self.coil = False
        return "ok off"

    def status(self):
        self._check()
        return "ok state " + ("on" if self.coil else "off")

    def close(self):
        self.closed = True


def _link(board, reconnect=None):
    said = []
    return handeye.MagnetLink(board, lambda t, secs=0: said.append(t), reconnect), said


def test_on_off_are_confirmed_by_the_board():
    link, said = _link(FakeBoard())
    assert link.set(True, settle_s=0) is True and link.on
    assert link.set(False, settle_s=0) is False and not link.on
    assert any("confirmed" in t for t in said)


def test_a_reset_on_switch_on_shows_off_not_on():
    link, said = _link(FakeBoard(reset_on_on=True))
    assert link.set(True, settle_s=0) is False and not link.on
    assert any("RESET" in t or "did NOT turn on" in t for t in said)


def test_the_30s_cap_expiring_is_noticed_on_the_next_poll():
    board = FakeBoard()
    link, said = _link(board)
    link.set(True, settle_s=0)
    board.coil = False
    board.s.lines.append("ok timeout off")
    assert link.poll() is False and not link.on
    assert any("timeout" in t for t in said)


def test_a_dead_board_reconnects_and_reports_off_never_crashes():
    fresh = FakeBoard()
    link, said = _link(FakeBoard(dead=True), reconnect=lambda: fresh)
    link.on = True
    assert link.set(True, settle_s=0) is None   # SystemExit from the driver is contained
    assert link.magnet is fresh and not link.on and any("reconnected" in t for t in said)


def test_unreachable_board_disables_the_magnet():
    def fail():
        raise SystemExit("no ESP32")
    link, said = _link(FakeBoard(dead=True), reconnect=fail)
    assert link.poll() is None and link.magnet is None and link.fault
