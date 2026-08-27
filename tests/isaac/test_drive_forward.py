"""Phase 2 regression: drive each imported robot forward >1 m in 5 s.

Two drive modes:
- "diff_drive" (Carter): DifferentialController + WheeledRobot, world-pose delta.
- "prismatic"  (XLeRobot): direct velocity target on `root_x_axis_joint`,
  joint-position delta. XLeRobot's URDF models a ManiSkill-style holonomic
  base via 3 root joints, so it must be imported with --fix-base and the
  prismatic joint is the actual "drive" DOF.
"""
from __future__ import annotations

import os
from pathlib import Path

import numpy as np
import pytest

from _robot_configs import ROBOT_CONFIGS  # tests/isaac/ is on sys.path via conftest


DRIVE_STEPS = 300
DT = 1.0 / 60.0  # 5 s of sim time


def _emit(name: str, line: str) -> None:
    out = f"/tmp/isaac_drive_test_{name}_result.txt"
    with open(out, "a") as f:
        f.write(line + "\n")
    print(line, flush=True)


@pytest.mark.parametrize("robot", ["carter", "xlerobot"])
def test_drive_forward(robot, request, world):
    cfg = ROBOT_CONFIGS[robot]
    usd_path: Path = request.getfixturevalue(f"{robot}_usd_path")

    # Truncate per-test result file.
    open(f"/tmp/isaac_drive_test_{robot}_result.txt", "w").close()
    _emit(robot, f"=== drive-forward {robot} ===")
    _emit(robot, f"usd={usd_path}")
    _emit(robot, f"drive_mode={cfg['drive_mode']}")

    if cfg["drive_mode"] == "diff_drive":
        delta_xy, end_z = _drive_diff_drive(robot, cfg, usd_path, world)
    elif cfg["drive_mode"] == "prismatic":
        delta_xy, end_z = _drive_prismatic(robot, cfg, usd_path, world)
    else:  # pragma: no cover
        pytest.fail(f"unknown drive_mode {cfg['drive_mode']!r}")

    _emit(robot, f"delta_xy={delta_xy:.4f}  end_z={end_z:.4f}")
    assert not np.isnan(delta_xy), "displacement was NaN"
    assert end_z < cfg["explosion_z"], (
        f"{robot} ended at z={end_z:.2f}, suggests PhysX explosion "
        f"(threshold {cfg['explosion_z']})"
    )
    assert delta_xy > 1.0, f"{robot} only moved {delta_xy:.3f} m (need > 1.0)"
    _emit(robot, f"drive-forward {robot}: OK")


def _drive_diff_drive(robot, cfg, usd_path, world):
    from isaacsim.robot.wheeled_robots.controllers.differential_controller import (
        DifferentialController,
    )
    from isaacsim.robot.wheeled_robots.robots import WheeledRobot

    prim_path = f"/World/{robot}"
    wb = WheeledRobot(
        prim_path=prim_path,
        name=robot,
        wheel_dof_names=cfg["wheel_dof_names"],
        create_robot=True,
        usd_path=str(usd_path),
        position=np.array(cfg["spawn_position"]),
    )
    world.scene.add(wb)
    world.reset()

    ctrl = DifferentialController(
        name="diff",
        wheel_radius=cfg["wheel_radius"],
        wheel_base=cfg["wheel_base"],
    )

    start_pos, _ = wb.get_world_pose()
    _emit(robot, f"start_xy=({start_pos[0]:.3f}, {start_pos[1]:.3f})")

    for _ in range(DRIVE_STEPS):
        action = ctrl.forward(np.array([cfg["linear_velocity"], 0.0]))
        wb.apply_wheel_actions(action)
        world.step(render=False)

    end_pos, _ = wb.get_world_pose()
    _emit(robot, f"end_xy=({end_pos[0]:.3f}, {end_pos[1]:.3f})  end_z={end_pos[2]:.3f}")
    return float(np.linalg.norm(end_pos[:2] - start_pos[:2])), float(end_pos[2])


def _drive_prismatic(robot, cfg, usd_path, world):
    from isaacsim.core.api.robots import Robot
    from isaacsim.core.utils.stage import add_reference_to_stage
    from isaacsim.core.utils.types import ArticulationAction

    prim_path = f"/World/{robot}"
    add_reference_to_stage(usd_path=str(usd_path), prim_path=prim_path)

    rb = Robot(prim_path=prim_path, name=robot, position=np.array(cfg["spawn_position"]))
    world.scene.add(rb)
    world.reset()

    dof_name = cfg["drive_dof_name"]
    dof_names = list(rb.dof_names)
    assert dof_name in dof_names, (
        f"{dof_name} not in articulation DOFs: {dof_names}"
    )
    dof_idx = dof_names.index(dof_name)
    _emit(robot, f"drive DOF: {dof_name} idx={dof_idx} (of {len(dof_names)})")

    start_q = float(rb.get_joint_positions()[dof_idx])
    _emit(robot, f"start_q={start_q:.3f}")

    v_target = cfg["target_velocity"]
    n_dof = len(dof_names)
    for _ in range(DRIVE_STEPS):
        velocities = np.zeros(n_dof)
        velocities[dof_idx] = v_target
        action = ArticulationAction(joint_velocities=velocities)
        rb.apply_action(action)
        world.step(render=False)

    end_q = float(rb.get_joint_positions()[dof_idx])
    pos, _ = rb.get_world_pose()
    end_z = float(pos[2])
    _emit(robot, f"end_q={end_q:.3f}  end_z={end_z:.3f}")
    return abs(end_q - start_q), end_z
