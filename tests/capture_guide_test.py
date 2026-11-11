"""scripts/capture_guide.py: the capture walkthrough moves through home -> pan +/-20 -> tilt
+/-10, spreads samples over the image, and only says "ready" when the view is usable."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
import capture_guide as G  # noqa: E402

WH = (1280, 720)
LIM = {"head_pan": (-120.0, 120.0), "head_tilt": (-70.0, 70.0)}


def _s(head, px):
    return {"head_deg": list(head), "px": list(px)}


def _centre(cell):
    x0, y0, x1, y1 = G.cell_box(cell, WH)
    return ((x0 + x1) / 2, (y0 + y1) / 2)


def test_first_sample_any_head_angle_and_ready_only_when_all_checks_pass():
    g = G.guide([], [3, 45], None, WH, LIM)
    assert g["title"].startswith("Find a HOME") and g["box"] and not g["ready"]
    cell = G.cell_of(_centre((0, 0)), WH)
    assert cell == (0, 0)
    good = {"px": _centre((0, 0)), "size_px": 34, "z": 0.30}
    assert G.guide([], [3, 45], good, WH, LIM)["ready"]
    assert not G.guide([], [3, 45], {**good, "z": 0.55}, WH, LIM)["ready"]       # first cell is NEAR
    assert not G.guide([], [3, 45], {**good, "size_px": 15}, WH, LIM)["ready"]


def test_box_moves_to_an_uncovered_cell_after_each_sample():
    samples = [_s((3, 45), _centre((0, 0)))]
    g = G.guide(samples, [3, 45], None, WH, LIM)
    assert g["box"] == G.cell_box((2, 0), WH)                 # HOME_PLAN's second cell
    assert g["title"] == "home: sample 2 of 6"


def test_home_done_sends_the_head_to_pan_plus_20_then_samples_there():
    home = [_s((3, 45), _centre(c[:2])) for c in G.HOME_PLAN]
    g = G.guide(home, [3, 45], None, WH, LIM)
    assert g["title"] == "Head to pan +20" and g["box"] is None
    assert "pan +23, tilt +45" in g["lines"][0]
    g = G.guide(home, [22, 44], None, WH, LIM)                # within 3 deg: sampling there
    assert g["title"] == "pan +20: sample 1 of 4" and g["box"]


def test_all_stages_done():
    s = [_s((0, 30), _centre(c[:2])) for c in G.HOME_PLAN]
    for _, (dp, dt) in G.OFFSETS:
        s += [_s((dp, 30 + dt), _centre(c[:2])) for c in G.OTHER_PLAN]
    g = G.guide(s, [0, 30], None, WH, LIM)
    assert g["done"] and all(r["have"] >= r["need"] for r in g["progress"])


def test_old_samples_without_pixels_still_count():
    s = [{"head_deg": [0, 30]} for _ in range(6)]
    assert G.guide(s, [0, 30], None, WH, LIM)["progress"][0]["have"] == 6
