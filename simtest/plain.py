"""A test's pass criteria in plain words, for the operator view's Tests tab.

Written from the thresholds rather than beside them, so the tab can't promise something
the test doesn't score: loosen `contacts: {eq: 0}` and "Never touches a walker" stops being
true, so its sentence returns None and the tab falls back to the raw check (`sentence()`).
A new metric or an unusual threshold reads a little rough until it gets a line here; it
never reads wrong. tests/simtest_spec_test.py holds every listed test to having sentences.
"""
from __future__ import annotations


def _cm(metres: float) -> str:
    return f"{round(metres * 100)} cm"


def _pct(fraction: float) -> str:
    return f"{round(fraction * 100)}%"


def _clearance_target(params: dict) -> str:
    if params.get("peers"):
        return "another robot" if params.get("actors") == [] else "a walker or another robot"
    if params.get("props"):
        return "a crate, barrel or pallet" if params.get("actors") == [] else "a walker or a crate"
    return "a walker"


def _s(seconds: float) -> str:
    return f"{seconds:g} s"


def _only_if(holds: bool, text: str) -> str | None:
    return text if holds else None


# (metric, value, op) -> f(threshold, metric params) -> sentence, or None if it doesn't hold.
SENTENCES = {
    ("clearance", "contacts", "eq"):
        lambda x, p: _only_if(x == 0, f"Never touches {_clearance_target(p)}"),
    ("clearance", "min_m", "gt"):
        lambda x, p: f"Stays more than {_cm(x)} away from {_clearance_target(p)}",
    ("goals", "success_rate", "gte"):
        lambda x, p: "Reaches every goal" if x >= 1 else f"Reaches at least {_pct(x)} of its goals",
    ("tasks", "success_rate", "gte"):
        lambda x, p: "Finishes the job it was given" if x >= 1 else f"Finishes at least {_pct(x)} of its jobs",
    ("map_contamination", "occupied_frac", "lte"):
        lambda x, p: f"At most {_pct(x)} of the walkers' floor ends up on the map as an obstacle",
    ("detection", "recall", "gte"):
        lambda x, p: f"Detects walkers in at least {_pct(x)} of the frames they are in view",
    ("goals", "max_detour_ratio", "lte"):
        lambda x, p: f"Drives no more than {x:g}x the straight-line distance to any goal",
    ("reaction", "encounters", "gte"):
        lambda x, p: "Actually meets a mover on the way" if x == 1
        else f"Meets a mover at least {int(x)} times on the way",
    ("reaction", "max_time_to_slow_s", "lte"):
        lambda x, p: f"Halves its speed within {_s(x)} of a mover becoming a threat, every time",
    ("reaction", "max_time_to_stop_s", "lte"):
        lambda x, p: f"Comes to a stop within {_s(x)} of a mover becoming a threat, every time",
    ("reaction", "max_brake_m", "lte"):
        lambda x, p: f"Needs no more than {_cm(x)} to brake for a mover",
    ("reaction", "min_ttc_s", "gt"):
        lambda x, p: f"Never comes within {_s(x)} of colliding with a mover",
    ("reaction", "min_gap_m", "gt"):
        lambda x, p: f"Keeps more than {_cm(x)} from a mover it is reacting to",
    ("pick_place", "detected", "gte"):
        lambda x, p: _only_if(x == 1, "The camera finds the block"),
    ("pick_place", "picked", "gte"):
        lambda x, p: _only_if(x == 1, "The magnet picks the block up"),
    ("pick_place", "held_at_end", "gte"):
        lambda x, p: _only_if(x == 1, "The block is still on the magnet when the test ends"),
    ("pick_place", "placed", "gte"):
        lambda x, p: _only_if(x == 1, "The block is set down on the marked area"),
    ("pick_place", "max_place_error_m", "lte"):
        lambda x, p: f"It lands within {_cm(x)} of the target",
    ("pick_place", "max_place_z_error_m", "lte"):
        lambda x, p: f"It rests at table height, to within {_cm(x)}",
    ("pick_place", "drops", "eq"):
        lambda x, p: _only_if(x == 0, "Never drops it on the way"),
    ("pick_place", "false_picks", "eq"):
        lambda x, p: _only_if(x == 0, "Never picks up the wrong package"),
}


def sentence(entry: dict, value: str, op: str, threshold) -> str | None:
    """One resolved metric entry's criterion in words, or None if it has none."""
    say = SENTENCES.get((entry["metric"], value, op))
    return say(threshold, entry.get("params") or {}) if say else None
