"""Pure helpers of scripts/head.py: calibrated limits and the per-tick head step (no lerobot)."""
import sys
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
import head  # noqa: E402

CAL = {"head_pan": SimpleNamespace(range_min=567, range_max=3412),
       "head_tilt": SimpleNamespace(range_min=847, range_max=2889)}


def test_limits_are_symmetric_about_mid_and_inside_the_stops():
    lim = head.limits(CAL)
    lo, hi = lim["head_pan"]
    assert lo == -hi and 120 < hi < 124      # (3412-567)/2 steps = ~125 deg, minus the 2 deg margin


def test_step_integrates_and_clamps():
    lim = head.limits(CAL)
    t = {"head_pan": 0.0, "head_tilt": 0.0}
    t = head.head_step(t, 1.0, -0.5, 90.0, 0.1, lim)
    assert t == {"head_pan": 9.0, "head_tilt": -4.5}
    for _ in range(100):
        t = head.head_step(t, 1.0, -1.0, 90.0, 0.1, lim)
    assert t["head_pan"] == lim["head_pan"][1] and t["head_tilt"] == lim["head_tilt"][0]


def test_zero_input_returns_an_equal_target():
    lim = head.limits(CAL)
    t = {"head_pan": 3.0, "head_tilt": -2.0}
    assert head.head_step(t, 0.0, 0.0, 90.0, 0.1, lim) == t


def test_head_axes_moves_one_axis_only():
    assert head.head_axes(0.8, 0.3) == (0.8 * 0.8, 0.0)       # mostly sideways: pan only
    assert head.head_axes(-0.2, -0.9) == (0.0, -0.9 * 0.9)    # mostly down: tilt only
    assert head.head_axes(-1.0, 0.0) == (-1.0, 0.0)           # full push is still full speed
    assert head.head_axes(0.0, 0.0) == (0.0, 0.0)


def test_servo_fault_names_the_bits():
    from status_line import servo_fault
    assert servo_fault(0b100000) == "overload"
    assert servo_fault(0b100100) == "overheat+overload"
    assert servo_fault(0b10000000) == "bit7"
