"""robot_0's magnet effector inside Isaac: stepped every physics step by spawn_warehouse.py.

Lives in its own module, not in spawn_warehouse.py, because that script boots a
SimulationApp at import and so cannot be imported by a test. Everything Isaac-specific
here (`pxr`, the robot articulation, the package prims' xform ops) is passed in or
imported lazily inside a method, which is what lets tests/magnet_runtime_test.py drive
this class with a fake robot and a stub `pxr`.

The grasp is simtest.magnet.MagnetSim: energized (pick_cmd "attach") and within capture
range of a package's plate, the package is SNAPPED onto the magnet and rides the arm;
de-energized ("detach"), it drops straight onto whatever is under it. No contact, no
compliance, no slip, no mass: a model for writing and scoring pick tests, not for
training a grasp (README.md §13).

WHERE THE MAGNET IS. Computed, not read. Forward kinematics (nodes/arm_kin.py, the chain
the pick controller solves against) from the SIMULATED JOINT POSITIONS, with the base
pose from the three root joints exactly as ground truth takes it, so it needs nothing
from USD. The old mock pick read the gripper link's USD world transform every step and
never confirmed that Isaac keeps that in sync with the physics. This does not depend on
it, and it checks itself: on the first step, and every CROSSCHECK_EVERY steps after, the
FK pose is compared with the arm link's USD transform and a disagreement over 2 cm is
logged (the first comparison always is). If the log is quiet, both ways of finding the
magnet agree; if not, it says by how much, which is everything a first live run needs
to know about frames and units.
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent


class MagnetRuntime:
    CROSSCHECK_EVERY = 300   # physics steps (~5 sim-s)
    CROSSCHECK_WARN_M = 0.02
    CROSSCHECK_WARNINGS = 5  # then stay quiet: a persistent disagreement is already known

    def __init__(self, stage, w: dict, package_ops: dict, pick_state: dict, robot_name: str,
                 spawn, *, emit, arm_link: str, arm_joints: list[str]):
        """w: the validated world (simtest.world); package_ops: {id: (translate_op,
        rotate_op)}; pick_state: {"data": last pick_cmd string, ...}; spawn: the robot's
        (x, y, z) spawn pose; arm_joints: the arm's joint names in kinematic order (the
        first five are the ones arm_kin knows)."""
        for p in (str(REPO_ROOT / "nodes"), str(REPO_ROOT)):
            if p not in sys.path:
                sys.path.insert(0, p)
        import arm_kin
        from simtest import magnet as magnet_lib

        self.m = magnet_lib
        self.emit = emit
        self.arm_link, self.arm_joints = arm_link, list(arm_joints)
        self.stage, self.ops, self.pick = stage, package_ops, pick_state
        self.robot_name, self.spawn = robot_name, spawn
        self.kin = arm_kin.ArmKin()
        self.sim = magnet_lib.MagnetSim(
            {p["id"]: {"pose": {"x": p["pose"][0], "y": p["pose"][1], "z": p["z"],
                                "yaw": p["pose"][2]},
                       "size": p["size"], "face": p["magnet_face"]}
             for p in w["packages"]},
            lambda x, y: magnet_lib.support_top_z(w, x, y))
        self.idx: list[int] | None = None
        self.n = 0
        self.warned = 0
        self.tcp = self.approach = None
        self.diag: dict = {"energized": False, "held": None, "nearest": None}

    def describe(self) -> str:
        return f"{len(self.ops)} package(s), tcp {self.m.TCP_OFFSET_M} from {self.arm_link}"

    def tcp_pose(self, robot) -> tuple[tuple, tuple]:
        """(TCP position, unit approach direction) in the map frame, from joint positions."""
        if self.idx is None:
            names = list(robot.dof_names)
            self.idx = [names.index(n) for n in self.arm_joints[:5]]
        q = robot.get_joint_positions()
        # Base pose as _WorldRuntime.tick reports it: spawn + the world-frame prismatic
        # joints, yaw from root_z_rotation (trig only, so it needs no wrapping here).
        base = (self.spawn[0] + float(q[0]), self.spawn[1] + float(q[1]), float(q[2]))
        pos_b, a_b = self.kin.tcp([float(q[i]) for i in self.idx])
        return (self.m.to_map(base, tuple(float(v) for v in pos_b)),
                self.m.dir_to_map(base, tuple(float(v) for v in a_b)))

    def step(self, robots: dict) -> None:
        from pxr import Gf

        self.tcp, self.approach = self.tcp_pose(robots[self.robot_name][0])
        if self.n % self.CROSSCHECK_EVERY == 0:
            self._cross_check()
        self.n += 1
        held_before = self.sim.held
        d = self.sim.step(self.tcp, self.approach, self.pick["data"] == "attach")
        for pid in d["moved"]:
            p = self.sim.pose(pid)
            t_op, r_op = self.ops[pid]
            t_op.Set(Gf.Vec3d(p["x"], p["y"], p["z"]))
            r_op.Set(Gf.Vec3f(0.0, 0.0, math.degrees(p["yaw"])))
        if d["held"] != held_before:
            if d["held"]:
                self.emit(f"  magnet: captured {d['held']} at tcp "
                          f"({self.tcp[0]:.3f}, {self.tcp[1]:.3f}, {self.tcp[2]:.3f})")
            else:
                p = self.sim.pose(held_before)
                self.emit(f"  magnet: released {held_before} at ({p['x']:.3f}, {p['y']:.3f}, "
                          f"{p['z']:.3f})")
        self.diag = d

    def _cross_check(self) -> None:
        from pxr import Gf, Usd, UsdGeom

        prim = self.stage.GetPrimAtPath(f"/World/{self.robot_name}/{self.arm_link}")
        if not prim.IsValid():
            if self.n == 0:
                self.emit(f"  magnet: FK-vs-USD cross-check skipped, no prim "
                          f"/World/{self.robot_name}/{self.arm_link}")
            return
        mat = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        usd_tcp = mat.Transform(Gf.Vec3d(*self.m.TCP_OFFSET_M))
        usd_a = mat.TransformDir(Gf.Vec3d(*self.m.APPROACH_LOCAL))
        norm = math.sqrt(sum(float(usd_a[i]) ** 2 for i in range(3))) or 1.0
        usd_a = tuple(float(usd_a[i]) / norm for i in range(3))
        gap = math.dist(self.tcp, tuple(float(usd_tcp[i]) for i in range(3)))
        cosang = max(-1.0, min(1.0, sum(a * b for a, b in zip(self.approach, usd_a))))
        ang = math.degrees(math.acos(cosang))
        bad = gap > self.CROSSCHECK_WARN_M
        if self.n == 0 or (bad and self.warned < self.CROSSCHECK_WARNINGS):
            self.warned += bad
            self.emit(
                f"  magnet tcp: FK ({self.tcp[0]:.3f}, {self.tcp[1]:.3f}, {self.tcp[2]:.3f}) "
                f"vs USD ({usd_tcp[0]:.3f}, {usd_tcp[1]:.3f}, {usd_tcp[2]:.3f}): "
                f"{gap * 100:.1f} cm apart, approach {ang:.1f} deg apart"
                + ("  <-- DISAGREE: check the TCP offset's axis/sign, the base pose "
                   "convention, or whether USD is being updated by the physics" if bad else ""))

    def truth(self) -> tuple[dict, dict]:
        """(packages, magnet) for /sim/ground_truth."""
        held = self.sim.held
        packages = {pid: {"x": round(p["x"], 4), "y": round(p["y"], 4), "z": round(p["z"], 4),
                          "yaw": round(p["yaw"], 4),
                          "held_by": self.robot_name if pid == held else None}
                    for pid, p in self.sim.poses().items()}
        r = lambda v: [round(float(c), 4) for c in v]  # noqa: E731
        magnet_state = {self.robot_name: {
            "energized": bool(self.diag.get("energized")), "held": held,
            "tcp": r(self.tcp) if self.tcp else None,
            "approach": r(self.approach) if self.approach else None,
            "nearest": self.diag.get("nearest")}}
        return packages, magnet_state
