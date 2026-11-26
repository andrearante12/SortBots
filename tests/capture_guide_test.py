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


def test_touch_mode_spreads_the_first_three_by_text_then_follows_the_reach_map():
    g = G.guide([], [3, 74], None, WH, LIM, mode="touch", marker_id=10)
    assert g["box"] is None and any("anywhere the magnet can touch" in l for l in g["lines"])
    g = G.guide([], [3, 74], None, WH, LIM, mode="touch", pending=True)
    assert g["title"] == "Touch the circle" and g["ready"] and g["box"] is None
    two = [_s((3, 74), (300, 300)), _s((3, 74), (700, 300))]
    g = G.guide(two, [3, 74], None, WH, LIM, mode="touch")
    assert any("triangle" in l for l in g["lines"]) and not any("book" in l for l in g["lines"])
    three = two + [_s((3, 74), (500, 600))]
    reach = [[x, y] for x in range(200, 1100, 50) for y in range(250, 650, 50)]
    g = G.guide(three, [3, 74], {"px": [1050, 590], "size_px": 22}, WH, LIM, mode="touch",
                marker_id=10, reach_px=reach)
    assert g["reach"] == reach and g["box"]
    cx, cy = (g["box"][0] + g["box"][2]) / 2, (g["box"][1] + g["box"][3]) / 2
    # the farthest reachable spot from the three samples: the reach's bottom-right corner
    assert G.pick_spot(reach, [s["px"] for s in three]) == [1050, 600]
    assert (cx, cy) == (1050, 600) or G._in(g["box"], [1050, 600])
    assert g["ready"]  # marker seen, in that box, 18+ px
    five = three + [_s((3, 74), (900, 300)), _s((3, 74), (300, 600))]
    assert any("book" in l for l in G.guide(five, [3, 74], None, WH, LIM, mode="touch", reach_px=reach)["lines"])


def test_touch_mode_keeps_guiding_extras_after_the_plan():
    s = [_s((0, 30), _centre(c[:2])) for c in G.HOME_PLAN]
    for _, (dp, dt) in G.OFFSETS:
        s += [_s((dp, 30 + dt), _centre(c[:2])) for c in G.OTHER_PLAN]
    g = G.guide(s, [0, 30], {"px": [600, 300], "size_px": 30}, WH, LIM, mode="touch")
    assert g["title"] == "Extra samples: 0 taken" and any("BOOK" in l for l in g["lines"]) and g["ready"]
    g = G.guide(s + [_s((0, 30), (100, 100))], [0, 30], None, WH, LIM, mode="touch")
    assert g["title"] == "Extra samples: 1 taken" and any("on the table" in l for l in g["lines"])
    g = G.guide(s, [0, 30], None, WH, LIM, mode="touch", pending=True)   # a touch after the plan
    assert g["title"] == "Touch the circle" and g["ready"]
    assert G.guide(s, [0, 30], None, WH, LIM)["title"] == "All samples taken"  # magnet mode as before
