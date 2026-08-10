#!/usr/bin/env python3
"""Pseudo-inverse IK over the XLeRobot arm chain. Optional, off the demo path.

Split out of `nodes/scripted_pick.py`, which now plays back recorded joint
angles from `configs/arm_poses.yaml` instead of solving. The reason is blunt:
`import torch` at that module's top level meant no launch file could ever start
it, because torch and pytorch-kinematics live only in the `lerobot` conda env
and this repo's hard rule is never to source ROS 2 in a conda shell. Playback
needs neither, so the pick/place demo runs on plain system python while this
module stays available for anything that genuinely needs a solve.

Both heavy imports are lazy (inside `ArmIKSolver.__init__`) so merely importing
this file is free — `scripted_pick.py --mode ik` is the only thing that
constructs the class.

IK: `pytorch_kinematics.PseudoInverseIK` over the arm chain from
`third_party/XLeRobot/.../xlerobot.urdf` (`base_link` -> `Fixed_Jaw_tip`, the
first/right arm). Full derivation notes, carried over verbatim because each one
cost a live investigation:

- The chain rooted at `base_link` unexpectedly still includes
  `root_z_rotation_joint` as its first DOF (URDF: root -> root_x -> root_y ->
  root_z -> base_link -> ...arm...; pytorch_kinematics' `root_link_name`
  doesn't trim joints strictly upstream of it). Pinning that DOF's limits to
  `(0, 0)` makes every solve arm-relative regardless of the robot's world yaw —
  verified: rotating that pinned DOF's value rotates the resulting
  end-effector position purely about Z with constant radius, confirming it
  behaves exactly like the base's own yaw.
- `PseudoInverseIK.joint_limits` is NOT a hard constraint on the returned
  solution — verified against the real installed library (pytorch-kinematics
  0.7.6): a solve returned angles outside the URDF's stated limits (e.g.
  Pitch's [-0.1, 3.45] range got a 4.33 rad answer) despite passing those same
  limits in. `solve()` below re-checks every retry's solution against the true
  limits and only accepts one that's actually valid, returning None (caller
  treats as failure) if none of the retries qualify.
- Gripper orientation is NOT solved for independently. All reach targets reuse
  the neutral/stow configuration's own (proven-reachable) orientation and only
  vary position — sidesteps guessing which way `Fixed_Jaw_tip`'s local frame
  points without a live session to check.

Reach budget, useful before burning a GPU session: summing the URDF link
origins from `Rotation` to `Fixed_Jaw_tip` gives roughly 0.50 m of arm fully
extended. Any dock standoff at or beyond that is unreachable by construction,
not merely untuned.

Standalone (from the `lerobot` conda env, or anywhere with torch installed):

    python3 nodes/arm_ik.py --print-neutral
    python3 nodes/arm_ik.py --offset 0 0 0.10
"""
from __future__ import annotations

import argparse
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent

URDF_PATH = (
    REPO_ROOT
    / "third_party"
    / "XLeRobot"
    / "simulation"
    / "Maniskill"
    / "assets"
    / "xlerobot"
    / "xlerobot.urdf"
)

# The five solved DOFs — the Jaw is commanded directly, never solved for.
IK_JOINT_NAMES = ["Rotation", "Pitch", "Elbow", "Wrist_Pitch", "Wrist_Roll"]
IK_NUM_RETRIES = 32


class ArmIKSolver:
    """Owns the pytorch_kinematics chain + IK solver. No rclpy dependency."""

    def __init__(self, urdf_path: Path = URDF_PATH):
        # Lazy on purpose: see the module docstring. Importing this file must
        # not drag torch in, or scripted_pick.py becomes unlaunchable again.
        import pytorch_kinematics as pk
        import torch

        self._torch = torch

        with open(urdf_path, "rb") as f:
            urdf_bytes = f.read()
        self.chain = pk.build_serial_chain_from_urdf(
            urdf_bytes, end_link_name="Fixed_Jaw_tip", root_link_name="base_link"
        )
        self.joint_names = self.chain.get_joint_parameter_names()
        assert self.joint_names[0] == "root_z_rotation_joint", (
            f"expected chain[0] == root_z_rotation_joint (pin-to-zero trick "
            f"depends on it), got {self.joint_names[0]!r} — URDF structure changed?"
        )
        assert self.joint_names[1:] == IK_JOINT_NAMES, (
            f"expected {IK_JOINT_NAMES} after the pinned root, got {self.joint_names[1:]}"
        )

        lo, hi = self.chain.get_joint_limits()
        self.true_lo = torch.as_tensor(lo, dtype=torch.float32).clone()
        self.true_hi = torch.as_tensor(hi, dtype=torch.float32).clone()
        # Pin root_z_rotation_joint to 0 so every solve is base_link-relative
        # regardless of the robot's world yaw (see module docstring).
        pinned_lo = self.true_lo.clone()
        pinned_hi = self.true_hi.clone()
        pinned_lo[0] = 0.0
        pinned_hi[0] = 0.0

        self.neutral_q = (
            torch.clamp(pinned_lo, -3.0, 3.0) + torch.clamp(pinned_hi, -3.0, 3.0)
        ) / 2.0
        self.neutral_q[0] = 0.0
        self.neutral_pose = self.chain.forward_kinematics(self.neutral_q)
        self.neutral_pos = self.neutral_pose.get_matrix()[0, :3, 3]

        self.ik_solver = pk.PseudoInverseIK(
            self.chain,
            joint_limits=torch.stack([pinned_lo, pinned_hi], dim=1),
            num_retries=IK_NUM_RETRIES,
        )
        self._pk = pk

    def neutral_arm_positions(self) -> list[float]:
        """5 joint angles (no Jaw) for the stow pose."""
        return [float(v) for v in self.neutral_q[1:]]

    def solve(self, offset_xyz: tuple[float, float, float]) -> list[float] | None:
        """IK for `neutral_pos + offset_xyz`, holding the neutral orientation.

        Returns 5 joint angles (no Jaw), or None if no retry's solution
        actually satisfies the true (unpinned) joint limits — see the
        `PseudoInverseIK.joint_limits` caveat in the module docstring.
        """
        torch = self._torch
        target_pos = self.neutral_pos + torch.tensor(offset_xyz, dtype=torch.float32)
        target = self._pk.Transform3d(
            rot=self.neutral_pose.get_matrix()[0, :3, :3].unsqueeze(0),
            pos=target_pos.unsqueeze(0),
        )
        sol = self.ik_solver.solve(target)
        if not bool(sol.converged_any[0]):
            return None
        for retry_idx in range(sol.solutions.shape[1]):
            if not bool(sol.converged[0, retry_idx]):
                continue
            q = sol.solutions[0, retry_idx]
            # q[0] (pinned root) is excluded from the true-limits check by
            # design — it's allowed to drift during optimization even though
            # we don't use its value (see module docstring).
            arm_q = q[1:]
            if bool(torch.all(arm_q >= self.true_lo[1:]) and torch.all(arm_q <= self.true_hi[1:])):
                return [float(v) for v in arm_q]
        return None


def main() -> int:
    parser = argparse.ArgumentParser(description="Bench the arm IK solver.")
    parser.add_argument("--urdf", type=Path, default=URDF_PATH)
    parser.add_argument("--print-neutral", action="store_true",
                        help="print the stow pose and its end-effector position")
    parser.add_argument("--offset", type=float, nargs=3, metavar=("X", "Y", "Z"),
                        help="solve for neutral_pos + this offset, in base_link frame")
    args = parser.parse_args()

    ik = ArmIKSolver(args.urdf)
    print(f"neutral q (no Jaw):       {[round(v, 4) for v in ik.neutral_arm_positions()]}")
    print(f"neutral ee pos (base_link): {[round(float(v), 4) for v in ik.neutral_pos]}")
    if args.offset:
        q = ik.solve(tuple(args.offset))
        print(f"solve({tuple(args.offset)}) -> "
              f"{[round(v, 4) for v in q] if q else 'FAILED (no retry within true limits)'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
