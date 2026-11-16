"""Offline: the XInput report parser (no pyusb, no pad)."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from xinput_pad import deadzone, parse  # noqa: E402


def pkt(b2=0, b3=0, lt=0, rt=0, lx=0, ly=0, rx=0, ry=0):
    def le(v):
        return (v & 0xFFFF).to_bytes(2, "little")
    return bytes([0x00, 0x14, b2, b3, lt, rt]) + le(lx) + le(ly) + le(rx) + le(ry) + bytes(6)


def test_buttons_and_triggers():
    st = parse(pkt(b2=0x10, b3=0x11, rt=255))  # START, LB + A
    assert st["buttons"]["START"] and st["buttons"]["LB"] and st["buttons"]["A"]
    assert not st["buttons"]["B"] and st["rt"] == 1.0 and st["lt"] == 0.0


def test_sticks_are_signed_up_positive():
    st = parse(pkt(lx=-32768, ly=32767, rx=16384, ry=-16384))
    assert st["lx"] == -1.0 and abs(st["ly"] - 1.0) < 1e-3
    assert abs(st["rx"] - 0.5) < 1e-3 and abs(st["ry"] + 0.5) < 1e-3


def test_ignores_non_input_reports():
    assert parse(bytes([0x01, 0x03, 0x00])) is None
    assert parse(b"") is None


def test_deadzone():
    assert deadzone(0.1) == 0.0 and deadzone(-0.05) == 0.0
    assert abs(deadzone(1.0) - 1.0) < 1e-9 and deadzone(-1.0) < 0


def test_status_line_fits_width_and_drops_from_the_right():
    from status_line import status_line
    args = ("CART", True, "x 26.0 y  0.0 z  5.8cm p  12", "L2", True, 7, True, "bus 11.7V 31C", "magnet ok on")
    full = status_line(200, *args)
    for tag in ("CART", "HELD", "x 26.0", "MAG ON   7s", "bus 11.7V", "magnet ok on"):
        assert tag in full
    import re
    plain = lambda t: re.sub(r"\x1b\[[0-9;]*m", "", t)
    narrow = plain(status_line(40, *args))
    assert len(narrow) <= 40 and narrow.startswith("CART | HELD")
    assert "magnet ok on" not in narrow          # lowest priority goes first
    assert "LIMIT" in status_line(200, "CART", False, "p", "L1", False, 0, False, "", "")
    assert "MAG off" in status_line(200, "CART", False, "p", "L1", False, 0, True, "", "")
