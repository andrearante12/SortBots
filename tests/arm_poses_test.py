#!/usr/bin/env python3
"""Unit tests for nodes/arm_poses.py — the pose-book loader.

Pure python: no ROS, no Isaac, no GPU, no torch. Runs anywhere in under a
second, which is the point — this is the half of the pick pipeline that can be
checked without a live session.

    python3 -m pytest tests/arm_poses_test.py
    python3 tests/arm_poses_test.py            # same, without pytest

The repo's only other tests are tests/isaac/ (needs a GPU + the Isaac venv) and
webui/tests/*.mjs (needs node + chrome), so this file establishes tests/ for
plain-python logic. Nothing in the repo restricts pytest collection.
"""
from __future__ import annotations

import sys
import textwrap
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "nodes"))

from arm_poses import (  # noqa: E402
    ARM_JOINT_NAMES,
    ArmPoseError,
    Step,
    format_capture,
    interpolate,
    load_pose_book,
    validate,
)

GOOD = """
joint_names: [Rotation, Pitch, Elbow, Wrist_Pitch, Wrist_Roll, Jaw]
limits:
  Rotation:    [-2.1, 2.1]
  Pitch:       [-0.1, 3.45]
  Elbow:       [-0.2, 3.14159]
  Wrist_Pitch: [-1.8, 1.8]
  Wrist_Roll:  [-3.14159, 3.14159]
  Jaw:         [0.0, 1.7]
defaults: {move_sec: 1.0, settle_sec: 0.5}
sequences:
  pick:
    - {name: stow, q: [0, 0, 0, 0, 0, 0]}
    - {name: grab, q: [0, 1.0, 1.0, 0, 0, 1.2], move_sec: 0.5}
    - {name: weld, grip: attach}
  place:
    - {name: drop, grip: detach}
"""


def write(tmp_path: Path, text: str) -> Path:
    p = tmp_path / "poses.yaml"
    p.write_text(textwrap.dedent(text))
    return p


# ----------------------------------------------------------------- loading


def test_loads_a_good_book(tmp_path):
    book = load_pose_book(write(tmp_path, GOOD))
    assert book.joint_names == ARM_JOINT_NAMES
    assert sorted(book.sequences) == ["pick", "place"]
    assert [s.name for s in book.sequence("pick")] == ["stow", "grab", "weld"]
    assert validate(book) == []


def test_defaults_apply_per_step(tmp_path):
    book = load_pose_book(write(tmp_path, GOOD))
    stow, grab, weld = book.sequence("pick")
    assert (stow.move_sec, stow.settle_sec) == (1.0, 0.5)   # both defaulted
    assert (grab.move_sec, grab.settle_sec) == (0.5, 0.5)   # move_sec overridden
    assert weld.q is None and weld.grip == "attach"


def test_duration_sums_move_and_settle(tmp_path):
    book = load_pose_book(write(tmp_path, GOOD))
    # (1.0+0.5) + (0.5+0.5) + (1.0+0.5)
    assert book.duration("pick") == pytest.approx(4.0)


def test_unknown_sequence_names_itself(tmp_path):
    book = load_pose_book(write(tmp_path, GOOD))
    with pytest.raises(ArmPoseError, match="no sequence 'nope'"):
        book.sequence("nope")


def test_missing_file(tmp_path):
    with pytest.raises(ArmPoseError, match="no pose file"):
        load_pose_book(tmp_path / "absent.yaml")


# -------------------------------------------------------------- rejections
# Every one of these is a mistake a hand-edited or mis-pasted capture can make.


@pytest.mark.parametrize("bad, expect", [
    # wrong joint count — the single most likely paste error
    ("sequences:\n  pick:\n    - {q: [0, 0, 0]}\n  place:\n    - {grip: detach}\n",
     "q must have 6 values"),
    # reordered joint names would silently drive the wrong DOFs
    ("joint_names: [Pitch, Rotation, Elbow, Wrist_Pitch, Wrist_Roll, Jaw]\n"
     "sequences:\n  pick:\n    - {grip: attach}\n  place:\n    - {grip: detach}\n",
     "joint_names must be exactly"),
    # a step that does nothing
    ("sequences:\n  pick:\n    - {name: noop}\n  place:\n    - {grip: detach}\n",
     "needs at least one of q / grip"),
    # typo'd grip verb
    ("sequences:\n  pick:\n    - {grip: grab}\n  place:\n    - {grip: detach}\n",
     "grip must be one of"),
    # a sequence task_manager will ask for and not find
    ("sequences:\n  pick:\n    - {grip: attach}\n",
     "missing required sequence 'place'"),
    # non-numeric angle
    ("sequences:\n  pick:\n    - {q: [0, 0, 0, 0, 0, 'x']}\n  place:\n    - {grip: detach}\n",
     "is not a number"),
    # negative dwell
    ("sequences:\n  pick:\n    - {grip: attach, settle_sec: -1}\n  place:\n    - {grip: detach}\n",
     "must be a non-negative number"),
    # a stray key is a typo, not something to ignore
    ("sequences:\n  pick:\n    - {grip: attach, mvoe_sec: 1}\n  place:\n    - {grip: detach}\n",
     "unknown keys"),
    ("sequences: {}\n", "must be a non-empty mapping"),
])
def test_rejects(tmp_path, bad, expect):
    with pytest.raises(ArmPoseError, match=expect):
        load_pose_book(write(tmp_path, bad))


def test_rejects_angle_outside_limits(tmp_path):
    # Pitch's upper limit is 3.45; 4.33 is the exact value the old IK path was
    # observed returning, which is why limits are enforced here at all.
    bad = GOOD.replace("q: [0, 1.0, 1.0, 0, 0, 1.2]", "q: [0, 4.33, 1.0, 0, 0, 1.2]")
    with pytest.raises(ArmPoseError, match=r"Pitch=4.3300 outside limit"):
        load_pose_book(write(tmp_path, bad))


def test_reports_every_problem_not_just_the_first(tmp_path):
    bad = """
    sequences:
      pick:
        - {q: [0, 0, 0]}
        - {grip: grab}
      place:
        - {name: noop}
    """
    with pytest.raises(ArmPoseError) as exc:
        load_pose_book(write(tmp_path, bad))
    msg = str(exc.value)
    assert "q must have 6 values" in msg
    assert "grip must be one of" in msg
    assert "needs at least one of q / grip" in msg


# ----------------------------------------------------------- interpolation


def test_interpolate_ends_exactly_on_target():
    frames = list(interpolate([0.0] * 6, [1.0] * 6, move_sec=1.0, hz=20))
    assert frames[-1] == [1.0] * 6
    assert len(frames) == 20


def test_interpolate_never_yields_the_start():
    # The arm is already at q_from; re-publishing it wastes a frame and, at the
    # start of a sequence, would look like a stall.
    frames = list(interpolate([0.0] * 6, [1.0] * 6, move_sec=1.0, hz=10))
    assert frames[0] != [0.0] * 6


def test_interpolate_is_monotonic_per_joint():
    frames = list(interpolate([0.0, 1.0] + [0.0] * 4, [1.0, 0.0] + [0.0] * 4, 1.0, 10))
    assert frames == sorted(frames, key=lambda f: f[0])
    assert frames == sorted(frames, key=lambda f: -f[1])


@pytest.mark.parametrize("move_sec", [0.0, -1.0, 0.01])
def test_degenerate_move_sec_still_reaches_target(move_sec):
    # A zero/negative ramp must not produce an empty generator: the step would
    # silently never happen.
    frames = list(interpolate([0.0] * 6, [0.5] * 6, move_sec, hz=20))
    assert frames == [[0.5] * 6]


# ---------------------------------------------------------- capture format


def test_format_capture_matches_the_dashboard():
    # Byte-identical to what webui/arm.js emits; webui/tests/arm_test.mjs
    # asserts the other direction by parsing the browser's output with this
    # loader. Both halves are hand-written, so both directions are checked.
    assert (format_capture([0.0, 1.047, -0.524, 0.0, 0.0, 1.2], "approach")
            == "    - {name: approach, q: [0.000, 1.047, -0.524, 0.000, 0.000, 1.200]}")


def test_captured_text_round_trips(tmp_path):
    captured = "\n".join(
        format_capture(q, name)
        for name, q in [("stow", [0.0] * 6), ("grab", [0.0, 1.0, 1.0, 0.0, 0.0, 1.2])]
    )
    # Built by concatenation, not a dedent'd triple-quote: format_capture emits
    # its own 4-space indent (so it pastes straight under a sequences: key) and
    # dedent would mangle exactly the thing under test.
    p = tmp_path / "poses.yaml"
    p.write_text("\n".join([
        "joint_names: [Rotation, Pitch, Elbow, Wrist_Pitch, Wrist_Roll, Jaw]",
        "sequences:",
        "  pick:",
        captured,
        "  place:",
        captured,
        "",
    ]))
    book = load_pose_book(p)
    assert [s.name for s in book.sequence("pick")] == ["stow", "grab"]
    assert book.sequence("pick")[1].q == pytest.approx([0.0, 1.0, 1.0, 0.0, 0.0, 1.2])


# --------------------------------------------------------- shipped config


def test_the_repo_pose_book_is_valid():
    book = load_pose_book(REPO_ROOT / "configs" / "arm_poses.yaml")
    assert validate(book) == []
    for name in ("pick", "place"):
        # task_manager.PICK_TIMEOUT_SEC is 30 s; scripted_pick warns at 25 s.
        assert book.duration(name) < 25.0, f"{name} would risk the pick timeout"


def test_validate_catches_a_hand_built_bad_book():
    from arm_poses import PoseBook

    book = PoseBook(sequences={"pick": [Step(name="noop")]})
    assert any("needs at least one of q / grip" in p for p in validate(book))


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v"]))
