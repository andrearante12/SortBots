#!/usr/bin/env python3
"""Joint-space arm pose sequences: load, validate, interpolate.

Deliberately free of rclpy, torch and pytorch_kinematics — the only import is
`yaml`. That is the whole point: `nodes/scripted_pick.py` used to carry a
module-scope `import torch`, which is why no launch file could ever start it
(torch and pytorch-kinematics live only in the `lerobot` conda env, not in
system python). Playing back recorded joint angles needs none of that, so the
node became launchable the moment IK stopped being on the critical path.

The recorded angles come from the dashboard's arm pad (`webui/arm.js`): jog the
arm, hit `cap`, paste the emitted block into `configs/arm_poses.yaml`. The
format `format_capture()` emits here and the format arm.js emits MUST stay
identical — `webui/tests/arm_test.mjs` round-trips the browser's text through
`load_pose_book()` to enforce exactly that.

Standalone check (no ROS, no GPU):

    python3 nodes/arm_poses.py configs/arm_poses.yaml
"""
from __future__ import annotations

import sys
from dataclasses import dataclass, field
from pathlib import Path

import yaml

# The contract with spawn_warehouse.ARM_JOINT_NAMES and webui/arm.js. Order is
# load-bearing: `_read_arm_cmd` looks joints up by name but the Isaac side
# applies them positionally, and a reordered list would silently drive the
# wrong DOFs.
ARM_JOINT_NAMES = [
    "Rotation", "Pitch", "Elbow", "Wrist_Pitch", "Wrist_Roll", "Jaw",
]
GRIPS = ("attach", "detach")

DEFAULT_MOVE_SEC = 1.2
DEFAULT_SETTLE_SEC = 0.4


class ArmPoseError(Exception):
    """Raised for a malformed pose file. Carries every problem, not the first."""


@dataclass
class Step:
    """One waypoint. At least one of `q` / `grip` is always set.

    `q` is a full six-joint target; `grip` publishes to `<robot>/pick_cmd` and
    drives the sim's mock weld. A step with both moves first, then grips.
    """

    name: str = ""
    q: list[float] | None = None
    grip: str | None = None
    move_sec: float = DEFAULT_MOVE_SEC
    settle_sec: float = DEFAULT_SETTLE_SEC

    @property
    def duration(self) -> float:
        return self.move_sec + self.settle_sec


@dataclass
class PoseBook:
    joint_names: list[str] = field(default_factory=lambda: list(ARM_JOINT_NAMES))
    limits: dict[str, tuple[float, float]] = field(default_factory=dict)
    sequences: dict[str, list[Step]] = field(default_factory=dict)

    def sequence(self, name: str) -> list[Step]:
        try:
            return self.sequences[name]
        except KeyError:
            known = ", ".join(sorted(self.sequences)) or "(none)"
            raise ArmPoseError(f"no sequence {name!r} in pose book; have: {known}")

    def duration(self, name: str) -> float:
        return sum(s.duration for s in self.sequence(name))


def _as_float_list(value, where: str, problems: list[str]) -> list[float] | None:
    if not isinstance(value, (list, tuple)):
        problems.append(f"{where}: q must be a list, got {type(value).__name__}")
        return None
    if len(value) != len(ARM_JOINT_NAMES):
        problems.append(
            f"{where}: q must have {len(ARM_JOINT_NAMES)} values "
            f"(one per {'/'.join(ARM_JOINT_NAMES)}), got {len(value)}"
        )
        return None
    out = []
    for i, v in enumerate(value):
        if isinstance(v, bool) or not isinstance(v, (int, float)):
            problems.append(f"{where}: q[{i}] is not a number ({v!r})")
            return None
        f = float(v)
        if f != f or f in (float("inf"), float("-inf")):
            problems.append(f"{where}: q[{i}] is not finite ({v!r})")
            return None
        out.append(f)
    return out


def load_pose_book(path: str | Path) -> PoseBook:
    """Parse and validate a pose file. Raises ArmPoseError listing every problem."""
    path = Path(path)
    try:
        raw = yaml.safe_load(path.read_text()) or {}
    except FileNotFoundError:
        raise ArmPoseError(f"no pose file at {path}")
    except yaml.YAMLError as exc:
        raise ArmPoseError(f"{path}: invalid YAML: {exc}")
    if not isinstance(raw, dict):
        raise ArmPoseError(f"{path}: top level must be a mapping")

    problems: list[str] = []
    book = PoseBook()

    names = raw.get("joint_names", list(ARM_JOINT_NAMES))
    if list(names) != ARM_JOINT_NAMES:
        problems.append(
            f"joint_names must be exactly {ARM_JOINT_NAMES} in that order "
            f"(the spawn_warehouse._read_arm_cmd contract), got {names}"
        )
    book.joint_names = list(ARM_JOINT_NAMES)

    # Limits are duplicated into the pose file (rather than parsed out of the
    # URDF) so a bad capture is rejected offline, with no ROS and no URDF path.
    for jname, bounds in (raw.get("limits") or {}).items():
        if jname not in ARM_JOINT_NAMES:
            problems.append(f"limits: unknown joint {jname!r}")
            continue
        if not isinstance(bounds, (list, tuple)) or len(bounds) != 2:
            problems.append(f"limits.{jname}: expected [lo, hi], got {bounds!r}")
            continue
        book.limits[jname] = (float(bounds[0]), float(bounds[1]))

    defaults = raw.get("defaults") or {}
    d_move = float(defaults.get("move_sec", DEFAULT_MOVE_SEC))
    d_settle = float(defaults.get("settle_sec", DEFAULT_SETTLE_SEC))

    sequences = raw.get("sequences")
    if not isinstance(sequences, dict) or not sequences:
        problems.append("sequences: must be a non-empty mapping")
        sequences = {}

    for seq_name, steps in sequences.items():
        if not isinstance(steps, list) or not steps:
            problems.append(f"sequences.{seq_name}: must be a non-empty list")
            continue
        parsed: list[Step] = []
        for idx, spec in enumerate(steps):
            where = f"sequences.{seq_name}[{idx}]"
            if not isinstance(spec, dict):
                problems.append(f"{where}: must be a mapping, got {type(spec).__name__}")
                continue
            unknown = set(spec) - {"name", "q", "grip", "move_sec", "settle_sec"}
            if unknown:
                problems.append(f"{where}: unknown keys {sorted(unknown)}")
            if "q" not in spec and "grip" not in spec:
                problems.append(f"{where}: needs at least one of q / grip")
                continue

            q = None
            if "q" in spec:
                q = _as_float_list(spec["q"], where, problems)
                if q is not None:
                    for jname, v in zip(ARM_JOINT_NAMES, q):
                        lo, hi = book.limits.get(jname, (None, None))
                        if lo is not None and not (lo <= v <= hi):
                            problems.append(
                                f"{where}: {jname}={v:.4f} outside limit [{lo}, {hi}]"
                            )

            grip = spec.get("grip")
            if grip is not None and grip not in GRIPS:
                problems.append(f"{where}: grip must be one of {GRIPS}, got {grip!r}")

            for key, dflt in (("move_sec", d_move), ("settle_sec", d_settle)):
                val = spec.get(key, dflt)
                if not isinstance(val, (int, float)) or isinstance(val, bool) or val < 0:
                    problems.append(f"{where}: {key} must be a non-negative number, got {val!r}")

            parsed.append(Step(
                name=str(spec.get("name", f"step_{idx}")),
                q=q,
                grip=grip,
                move_sec=float(spec.get("move_sec", d_move) or 0.0),
                settle_sec=float(spec.get("settle_sec", d_settle) or 0.0),
            ))
        book.sequences[seq_name] = parsed

    for required in ("pick", "place"):
        if required not in book.sequences:
            problems.append(f"sequences: missing required sequence {required!r}")

    if problems:
        raise ArmPoseError(f"{path}: " + "; ".join(problems))
    return book


def validate(book: PoseBook) -> list[str]:
    """Re-check an in-memory book. Returns problems; empty means good."""
    problems: list[str] = []
    if book.joint_names != ARM_JOINT_NAMES:
        problems.append(f"joint_names must be {ARM_JOINT_NAMES}")
    for seq_name, steps in book.sequences.items():
        for idx, step in enumerate(steps):
            where = f"{seq_name}[{idx}]"
            if step.q is None and step.grip is None:
                problems.append(f"{where}: needs at least one of q / grip")
            if step.q is not None and len(step.q) != len(ARM_JOINT_NAMES):
                problems.append(f"{where}: q must have {len(ARM_JOINT_NAMES)} values")
            if step.grip is not None and step.grip not in GRIPS:
                problems.append(f"{where}: bad grip {step.grip!r}")
    return problems


def interpolate(q_from, q_to, move_sec: float, hz: float):
    """Yield intermediate poses from q_from to q_to, ending exactly on q_to.

    Ramping matters more than it looks. The arm joints are position drives at
    stiffness 1000 / max_force 1000 (configs/physics_overrides/xlerobot.json),
    so writing a new target in one step commands a near-instantaneous slew. The
    previous publish-then-sleep approach did exactly that; it survives only
    because the package is teleport-welded to the gripper rather than actually
    held. Interpolating costs nothing, looks far better on video, and is the
    difference between working and flinging the box the day the package becomes
    a rigid body.

    q_from is never yielded (it is where the arm already is); q_to always is,
    so a zero or negative move_sec degenerates to a single immediate step.
    """
    steps = int(max(0.0, move_sec) * hz)
    for i in range(1, steps):
        t = i / steps
        yield [a + (b - a) * t for a, b in zip(q_from, q_to)]
    yield list(q_to)


def format_capture(q, name: str = "pose") -> str:
    """One YAML list entry, as `webui/arm.js` emits it.

    Kept byte-identical to the browser's output on purpose — arm_test.mjs
    round-trips the captured text through load_pose_book(), so a divergence
    here fails that test rather than surfacing as a confusing parse error
    hours later with a pasted config.
    """
    return f"    - {{name: {name}, q: [{', '.join(f'{float(v):.3f}' for v in q)}]}}"


def main(argv: list[str] | None = None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    path = Path(argv[0]) if argv else Path(__file__).resolve().parents[1] / "configs" / "arm_poses.yaml"
    try:
        book = load_pose_book(path)
    except ArmPoseError as exc:
        print(f"INVALID  {exc}", file=sys.stderr)
        return 1
    print(f"OK  {path}")
    for name in sorted(book.sequences):
        steps = book.sequences[name]
        print(f"  {name}: {len(steps)} steps, {book.duration(name):.1f}s"
              f"  ({', '.join(s.name for s in steps)})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
