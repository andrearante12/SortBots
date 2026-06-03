"""OmniGraph builders for per-robot ROS 2 publishers (Phase 3).

`build_robot_graphs(...)` adds three single-purpose Action Graphs under a
robot's prim:

- `<robot_prim>/OdometryGraph` — publishes `<namespace>/odom`
  (`nav_msgs/Odometry`), driven by `IsaacComputeOdometry` on the robot's
  chassis link.
- `<robot_prim>/CameraGraph`   — one render product feeds two
  `ROS2CameraHelper` nodes (`<namespace>/camera/rgb`,
  `<namespace>/camera/depth`).
- `<robot_prim>/TfGraph`       — full kinematic TF tree to
  `<namespace>/tf`. Standard multi-robot convention; nav stacks can
  remap `/robot_0/tf` → `/tf` downstream.

Module-level imports are stdlib-only. All `isaacsim` / `omni` / `pxr`
imports are lazy and happen inside `build_robot_graphs` so this file is
importable from a `SimulationApp`-free context (for documentation and
introspection).

This module mirrors the canonical bridge tests:
- `~/isaacsim/venv/.../isaacsim.ros2.bridge/.../tests/test_ros2_odometry.py:107-145`
- `~/isaacsim/venv/.../isaacsim.ros2.bridge/.../tests/test_camera.py:62-180`
"""
from __future__ import annotations

from typing import Tuple


def build_robot_graphs(
    robot_prim_path: str,
    chassis_subpath: str,
    camera_prim_path: str,
    namespace: str,
    odom_topic: str = "odom",
    rgb_topic: str = "camera/rgb",
    depth_topic: str = "camera/depth",
    tf_topic: str = "tf",
    rgb_resolution: Tuple[int, int] = (640, 480),
) -> None:
    """Author Odometry + Camera + TF Action Graphs for a single robot.

    Parameters
    ----------
    robot_prim_path : str
        Absolute USD path of the robot's root prim, e.g. ``/World/robot_0``.
    chassis_subpath : str
        Subpath under ``robot_prim_path`` whose world-pose is treated as the
        odometry source — typically ``"base_link"`` for the XLeRobot
        holonomic-base URDF.
    camera_prim_path : str
        Absolute USD path of the camera prim, e.g.
        ``/World/robot_0/head_camera_rgb_optical_frame/cam``.
    namespace : str
        ROS 2 node namespace (e.g. ``"robot_0"``). Prepended to every
        published topic by the bridge.
    odom_topic, rgb_topic, depth_topic, tf_topic : str
        Topic names *under* the namespace.
    rgb_resolution : (width, height)
        Render-product resolution. The same render product feeds both RGB
        and depth helpers.
    """
    import omni.graph.core as og
    import usdrt.Sdf

    chassis_prim_path = f"{robot_prim_path}/{chassis_subpath}"
    width, height = rgb_resolution
    odom_frame_id = f"{namespace}/odom"
    chassis_frame_id = f"{namespace}/base_link"
    camera_frame_id = f"{namespace}/camera_optical"

    _build_odometry_graph(
        og=og,
        usdrt=usdrt,
        graph_path=f"{robot_prim_path}/OdometryGraph",
        chassis_prim_path=chassis_prim_path,
        namespace=namespace,
        topic=odom_topic,
        odom_frame_id=odom_frame_id,
        chassis_frame_id=chassis_frame_id,
    )

    _build_camera_graph(
        og=og,
        usdrt=usdrt,
        graph_path=f"{robot_prim_path}/CameraGraph",
        camera_prim_path=camera_prim_path,
        namespace=namespace,
        rgb_topic=rgb_topic,
        depth_topic=depth_topic,
        width=width,
        height=height,
        frame_id=camera_frame_id,
    )

    _build_tf_graph(
        og=og,
        usdrt=usdrt,
        graph_path=f"{robot_prim_path}/TfGraph",
        target_prim_path=robot_prim_path,
        parent_prim_path=chassis_prim_path,
        namespace=namespace,
        topic=tf_topic,
    )


def _build_odometry_graph(
    *,
    og,
    usdrt,
    graph_path: str,
    chassis_prim_path: str,
    namespace: str,
    topic: str,
    odom_frame_id: str,
    chassis_frame_id: str,
) -> None:
    """Mirrors test_ros2_odometry.py:107-145 with namespacing added."""
    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": graph_path, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("ComputeOdometry", "isaacsim.core.nodes.IsaacComputeOdometry"),
                ("PublishOdometry", "isaacsim.ros2.bridge.ROS2PublishOdometry"),
            ],
            keys.SET_VALUES: [
                ("ComputeOdometry.inputs:chassisPrim", [usdrt.Sdf.Path(chassis_prim_path)]),
                ("PublishOdometry.inputs:topicName", topic),
                ("PublishOdometry.inputs:nodeNamespace", namespace),
                ("PublishOdometry.inputs:odomFrameId", odom_frame_id),
                ("PublishOdometry.inputs:chassisFrameId", chassis_frame_id),
                ("PublishOdometry.inputs:publishRawVelocities", False),
            ],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "ComputeOdometry.inputs:execIn"),
                ("ComputeOdometry.outputs:execOut", "PublishOdometry.inputs:execIn"),
                ("ComputeOdometry.outputs:position", "PublishOdometry.inputs:position"),
                ("ComputeOdometry.outputs:orientation", "PublishOdometry.inputs:orientation"),
                ("ComputeOdometry.outputs:linearVelocity", "PublishOdometry.inputs:linearVelocity"),
                ("ComputeOdometry.outputs:angularVelocity", "PublishOdometry.inputs:angularVelocity"),
                ("ReadSimTime.outputs:simulationTime", "PublishOdometry.inputs:timeStamp"),
            ],
        },
    )


def _build_camera_graph(
    *,
    og,
    usdrt,
    graph_path: str,
    camera_prim_path: str,
    namespace: str,
    rgb_topic: str,
    depth_topic: str,
    width: int,
    height: int,
    frame_id: str,
) -> None:
    """Mirrors test_camera.py:62-180, RGB + depth only.

    One render product feeds two ROS2CameraHelper nodes. The bridge picks
    the correct annotator from `inputs:type` (`distance_to_image_plane`
    for depth).
    """
    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": graph_path, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("CreateRenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                ("RGBPublish", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("DepthPublish", "isaacsim.ros2.bridge.ROS2CameraHelper"),
            ],
            keys.SET_VALUES: [
                ("CreateRenderProduct.inputs:cameraPrim", [usdrt.Sdf.Path(camera_prim_path)]),
                ("CreateRenderProduct.inputs:width", width),
                ("CreateRenderProduct.inputs:height", height),
                ("RGBPublish.inputs:type", "rgb"),
                ("RGBPublish.inputs:topicName", rgb_topic),
                ("RGBPublish.inputs:nodeNamespace", namespace),
                ("RGBPublish.inputs:frameId", frame_id),
                ("RGBPublish.inputs:resetSimulationTimeOnStop", True),
                ("DepthPublish.inputs:type", "depth"),
                ("DepthPublish.inputs:topicName", depth_topic),
                ("DepthPublish.inputs:nodeNamespace", namespace),
                ("DepthPublish.inputs:frameId", frame_id),
                ("DepthPublish.inputs:resetSimulationTimeOnStop", True),
            ],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "CreateRenderProduct.inputs:execIn"),
                ("CreateRenderProduct.outputs:execOut", "RGBPublish.inputs:execIn"),
                ("CreateRenderProduct.outputs:execOut", "DepthPublish.inputs:execIn"),
                ("CreateRenderProduct.outputs:renderProductPath", "RGBPublish.inputs:renderProductPath"),
                ("CreateRenderProduct.outputs:renderProductPath", "DepthPublish.inputs:renderProductPath"),
            ],
        },
    )


def _build_tf_graph(
    *,
    og,
    usdrt,
    graph_path: str,
    target_prim_path: str,
    parent_prim_path: str,
    namespace: str,
    topic: str,
) -> None:
    """Publish the kinematic TF tree under the robot's namespace.

    Standard ROS 2 multi-robot pattern: each robot publishes to
    `<namespace>/tf`. Downstream nav stacks remap to `/tf` if they expect
    the conventional topic name.
    """
    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": graph_path, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("PublishTf", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
            ],
            keys.SET_VALUES: [
                ("PublishTf.inputs:targetPrims", [usdrt.Sdf.Path(target_prim_path)]),
                ("PublishTf.inputs:parentPrim", [usdrt.Sdf.Path(parent_prim_path)]),
                ("PublishTf.inputs:topicName", topic),
                ("PublishTf.inputs:nodeNamespace", namespace),
            ],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "PublishTf.inputs:execIn"),
                ("ReadSimTime.outputs:simulationTime", "PublishTf.inputs:timeStamp"),
            ],
        },
    )
