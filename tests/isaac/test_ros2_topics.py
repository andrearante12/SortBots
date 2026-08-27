"""Phase 3 regression: 2 XLeRobot instances publish distinct ROS 2 topics.

Mechanics:

1. Reference the warehouse + 2 XLeRobots onto the `world` fixture's stage.
2. Attach a Camera to each robot at 320x240 / 15 Hz (small for CI VRAM).
3. Build odom / camera / tf OmniGraphs per robot via
   `scripts/_ros2_graphs.build_robot_graphs`.
4. Tick Kit a few times so the bridge finishes registering node types,
   then `timeline.play()`.
5. Inside the SAME Kit process, `import rclpy` (the bridge's bundled rclpy
   is on sys.path once the extension is active) and create subscribers
   for all 6 expected topics + per-robot /tf.
6. Step the sim for up to 8 wall-clock seconds, breaking early once every
   topic has received >= 1 message.
7. Assert:
   - Every expected topic has at least one message.
   - Frame IDs differ between robot_0 and robot_1 on both odom and the
     RGB camera.
   - robot_0's odom shows > 0.01 m of motion across the test window
     (circle-drive is applied).

Per-test result file: `/tmp/isaac_ros2_topics_result.txt`.
"""
from __future__ import annotations

import os
import sys
import time
from pathlib import Path

import numpy as np
import pytest

SCRIPTS_DIR = Path(__file__).resolve().parents[2] / "scripts"
sys.path.insert(0, str(SCRIPTS_DIR))


def _emit(line: str) -> None:
    out = "/tmp/isaac_ros2_topics_result.txt"
    with open(out, "a") as f:
        f.write(line + "\n")
    print(line, flush=True)


def _ensure_warehouse_usd() -> Path:
    """Subprocess `build_warehouse.py` if scenes/warehouse_v0.usd is missing."""
    from _robot_configs import REPO_ROOT  # type: ignore

    out = REPO_ROOT / "scenes" / "warehouse_v0.usd"
    if out.is_file():
        return out

    import subprocess

    cmd = [
        sys.executable,
        str(SCRIPTS_DIR / "build_warehouse.py"),
        "--out",
        str(out),
    ]
    r = subprocess.run(cmd, capture_output=True, text=True, timeout=180)
    if r.returncode != 0 or not out.is_file():
        raise RuntimeError(
            f"build_warehouse.py failed (rc={r.returncode}).\n"
            f"stdout:\n{r.stdout}\nstderr:\n{r.stderr}"
        )
    return out


def _drive_velocities(name: str, n_dof: int) -> np.ndarray:
    v = np.zeros(n_dof)
    v[0] = 0.15  # root_x_axis_joint forward
    v[2] = 0.10 if name == "robot_0" else -0.10  # root_z_rotation_joint
    return v


# Skip the test if ROS 2 is sourced — the bundled rclpy clashes with the
# system rclpy and the process tends to segfault on shutdown.
pytestmark = pytest.mark.skipif(
    os.environ.get("AMENT_PREFIX_PATH") is not None,
    reason="ROS 2 is sourced in the launching shell; bundled rclpy would clash.",
)


def test_ros2_topics(world, xlerobot_usd_path):
    open("/tmp/isaac_ros2_topics_result.txt", "w").close()
    _emit("=== test_ros2_topics ===")
    try:
        _run_test(world, xlerobot_usd_path)
    except BaseException as e:
        import traceback

        tb = traceback.format_exc()
        _emit(f"!!! EXCEPTION: {type(e).__name__}: {e}")
        for line in tb.splitlines():
            _emit(f"  {line}")
        raise


def _run_test(world, xlerobot_usd_path):

    warehouse_usd = _ensure_warehouse_usd()
    _emit(f"warehouse_usd={warehouse_usd}")
    _emit(f"xlerobot_usd={xlerobot_usd_path}")

    import omni.timeline
    from isaacsim.core.api.robots import Robot
    from isaacsim.core.utils.stage import add_reference_to_stage
    from isaacsim.core.utils.types import ArticulationAction
    from isaacsim.sensors.camera import Camera

    from _robot_configs import REPO_ROOT  # type: ignore  # noqa: F401
    from _ros2_graphs import build_robot_graphs  # type: ignore

    from build_warehouse import SPAWN_POSITIONS  # type: ignore

    add_reference_to_stage(usd_path=str(warehouse_usd), prim_path="/World/Warehouse")

    robots: dict[str, tuple[Robot, Camera, str]] = {}
    for name, pos in SPAWN_POSITIONS.items():
        prim_path = f"/World/{name}"
        add_reference_to_stage(usd_path=str(xlerobot_usd_path), prim_path=prim_path)
        robot = Robot(prim_path=prim_path, name=name, position=np.array(pos))
        world.scene.add(robot)
        cam_path = f"{prim_path}/head_camera_rgb_optical_frame/cam"
        cam = Camera(prim_path=cam_path, resolution=(320, 240), frequency=15)
        robots[name] = (robot, cam, cam_path)

    world.reset()
    for name, (_, cam, _) in robots.items():
        cam.initialize()
        cam.set_local_pose(orientation=np.array([0.0, 1.0, 0.0, 0.0]))

    for name, (_, _, cam_path) in robots.items():
        build_robot_graphs(
            robot_prim_path=f"/World/{name}",
            chassis_subpath="base_link",
            camera_prim_path=cam_path,
            namespace=name,
            rgb_resolution=(320, 240),
        )
        _emit(f"graphs built for {name}")

    # Let the bridge finish registering node types; then start the timeline.
    import omni.kit.app

    app = omni.kit.app.get_app()
    for _ in range(3):
        app.update()
    omni.timeline.get_timeline_interface().play()

    # Same-process rclpy. Confirm we picked up the bundled rclpy.
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    from tf2_msgs.msg import TFMessage

    _emit(f"rclpy.__file__={rclpy.__file__}")

    if not rclpy.ok():
        rclpy.init()
    node = Node("phase3_topic_tester")

    received: dict[str, list] = {}

    def make_cb(key):
        def cb(msg):
            received.setdefault(key, []).append(msg)

        return cb

    expected = [
        ("/robot_0/odom", Odometry),
        ("/robot_0/camera/rgb", Image),
        ("/robot_0/camera/depth", Image),
        ("/robot_0/tf", TFMessage),
        ("/robot_1/odom", Odometry),
        ("/robot_1/camera/rgb", Image),
        ("/robot_1/camera/depth", Image),
        ("/robot_1/tf", TFMessage),
    ]
    for topic, msg_t in expected:
        node.create_subscription(msg_t, topic, make_cb(topic), 10)

    dof_counts = {name: len(robot.dof_names) for name, (robot, _, _) in robots.items()}
    _emit(f"dof_counts={dof_counts}")

    try:
        # Drive the robots while spinning rclpy. Wall-clock budget: 12 s.
        deadline = time.time() + 12.0
        step = 0
        while time.time() < deadline:
            for name, (robot, _, _) in robots.items():
                v = _drive_velocities(name, dof_counts[name])
                robot.apply_action(ArticulationAction(joint_velocities=v))
            world.step(render=True)
            rclpy.spin_once(node, timeout_sec=0.001)
            step += 1
            if all(t in received for t, _ in expected):
                # Give it a few extra ticks so robot_0 odom has range to move.
                for _ in range(30):
                    for name, (robot, _, _) in robots.items():
                        v = _drive_velocities(name, dof_counts[name])
                        robot.apply_action(ArticulationAction(joint_velocities=v))
                    world.step(render=True)
                    rclpy.spin_once(node, timeout_sec=0.001)
                break

        for topic, _ in expected:
            count = len(received.get(topic, []))
            _emit(f"  {topic}: {count} messages")

        missing = [t for t, _ in expected if not received.get(t)]
        assert not missing, f"no messages on: {missing}"

        # Distinct frame_ids between robot_0 and robot_1.
        r0_odom_frame = received["/robot_0/odom"][0].header.frame_id
        r1_odom_frame = received["/robot_1/odom"][0].header.frame_id
        _emit(f"  odom frame_ids: r0={r0_odom_frame!r}  r1={r1_odom_frame!r}")
        assert r0_odom_frame != r1_odom_frame, "odom frame_ids collide"

        r0_rgb_frame = received["/robot_0/camera/rgb"][0].header.frame_id
        r1_rgb_frame = received["/robot_1/camera/rgb"][0].header.frame_id
        _emit(f"  rgb frame_ids:  r0={r0_rgb_frame!r}  r1={r1_rgb_frame!r}")
        assert r0_rgb_frame != r1_rgb_frame, "rgb frame_ids collide"

        # Motion: robot_0 odom moved between first and last sample.
        p0 = received["/robot_0/odom"][0].pose.pose.position
        pN = received["/robot_0/odom"][-1].pose.pose.position
        delta = ((pN.x - p0.x) ** 2 + (pN.y - p0.y) ** 2) ** 0.5
        _emit(f"  robot_0 odom motion: delta={delta:.4f} m over {len(received['/robot_0/odom'])} samples")
        assert delta > 0.01, f"robot_0 odom didn't move enough: {delta}"

        _emit("test_ros2_topics: OK")
    finally:
        try:
            node.destroy_node()
        finally:
            if rclpy.ok():
                rclpy.shutdown()
