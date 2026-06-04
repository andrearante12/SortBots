"""OmniGraph builders for per-robot ROS 2 publishers + subscribers.

`build_robot_graphs(...)` adds five Action Graphs under a robot's prim:

- `<robot_prim>/OdometryGraph` — publishes `<namespace>/odom`
  (`nav_msgs/Odometry`) from `IsaacComputeOdometry` on `base_link`.
- `<robot_prim>/CameraGraph`   — one render product feeds three
  `ROS2*Helper` nodes (`<namespace>/camera/rgb`,
  `<namespace>/camera/depth`, `<namespace>/camera/camera_info`).
- `<robot_prim>/TfGraph`       — full kinematic TF tree to
  `<namespace>/tf`.
- `<robot_prim>/CmdVelGraph`   — `ROS2SubscribeTwist` on
  `<namespace>/cmd_vel`; `spawn_warehouse.py` reads the output
  attributes each tick (no script-node inside the graph).
- `<robot_prim>/ImuGraph`      — `IsaacReadIMU` on a sensor prim under
  `base_link` → `ROS2PublishImu` to `<namespace>/imu`.

Module-level imports are stdlib-only. All `isaacsim` / `omni` / `pxr`
imports are lazy and happen inside `build_robot_graphs` so this file is
importable from a `SimulationApp`-free context.

References the canonical bridge samples:
- `~/isaacsim/.../isaacsim.ros2.bridge/.../tests/test_ros2_odometry.py:107-145`
- `~/isaacsim/.../isaacsim.ros2.bridge/.../tests/test_camera.py:62-180`
- `~/isaacsim/.../isaacsim.ros2.bridge/.../tests/test_subscribers.py`
- `~/isaacsim/.../isaacsim.sensors.physics/.../tests/test_imu_sensor_ogn.py`
"""
from __future__ import annotations

from typing import Optional, Tuple


def build_robot_graphs(
    robot_prim_path: str,
    chassis_subpath: str,
    camera_prim_path: str,
    namespace: str,
    odom_topic: str = "odom",
    rgb_topic: str = "camera/rgb",
    depth_topic: str = "camera/depth",
    camera_info_topic: str = "camera/camera_info",
    tf_topic: str = "tf",
    cmd_vel_topic: str = "cmd_vel",
    imu_prim_path: Optional[str] = None,
    imu_topic: str = "imu",
    rgb_resolution: Tuple[int, int] = (640, 480),
    base_to_camera: Optional[Tuple[Tuple[float, float, float], Tuple[float, float, float, float]]] = None,
    base_to_imu: Optional[Tuple[Tuple[float, float, float], Tuple[float, float, float, float]]] = None,
) -> None:
    """Author Odometry + Camera + TF + CmdVel (+ optional IMU) graphs.

    Parameters
    ----------
    robot_prim_path : str
        Absolute USD path of the robot's root prim, e.g. ``/World/robot_0``.
    chassis_subpath : str
        Subpath under ``robot_prim_path`` whose world-pose is the odometry
        source — typically ``"base_link"`` for the XLeRobot URDF.
    camera_prim_path : str
        Absolute USD path of the camera prim, e.g.
        ``/World/robot_0/head_camera_rgb_optical_frame/cam``.
    namespace : str
        ROS 2 node namespace (e.g. ``"robot_0"``).
    imu_prim_path : str or None
        Absolute USD path of an `IsaacImuSensor` prim. When ``None``
        (default), the IMU graph is NOT authored — preserves Phase 3
        behavior for callers that don't have an IMU yet.
    rgb_resolution : (width, height)
        Render-product resolution shared by RGB / depth / camera_info.
    base_to_camera, base_to_imu : ((tx,ty,tz), (qx,qy,qz,qw)) or None
        Fixed transforms from ``<ns>/base_link`` to the camera optical frame
        / IMU frame, broadcast on ``<ns>/tf`` so an external SLAM consumer can
        connect the RGB-D + IMU streams to the odom→base_link chain. When
        ``None`` the corresponding extrinsic is not published (Phase 3
        behavior). Rotation is the (x,y,z,w) quaternion the bridge's raw TF
        node expects (IJKR).
    """
    import omni.graph.core as og
    import usdrt.Sdf

    chassis_prim_path = f"{robot_prim_path}/{chassis_subpath}"
    width, height = rgb_resolution
    odom_frame_id = f"{namespace}/odom"
    chassis_frame_id = f"{namespace}/base_link"
    camera_frame_id = f"{namespace}/camera_optical"
    imu_frame_id = f"{namespace}/imu_link"

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
        camera_info_topic=camera_info_topic,
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

    _build_cmd_vel_graph(
        og=og,
        graph_path=f"{robot_prim_path}/CmdVelGraph",
        namespace=namespace,
        topic=cmd_vel_topic,
    )

    if imu_prim_path is not None:
        _build_imu_graph(
            og=og,
            usdrt=usdrt,
            graph_path=f"{robot_prim_path}/ImuGraph",
            imu_prim_path=imu_prim_path,
            namespace=namespace,
            topic=imu_topic,
            frame_id=imu_frame_id,
        )

    # Fixed base_link -> {camera_optical, imu_link} extrinsics on <ns>/tf, so
    # the prefixed frames the data streams are stamped with actually exist in
    # the TF tree and connect to odom->base_link.
    extrinsics = []
    if base_to_camera is not None:
        extrinsics.append((chassis_frame_id, camera_frame_id, base_to_camera))
    if base_to_imu is not None:
        extrinsics.append((chassis_frame_id, imu_frame_id, base_to_imu))
    if extrinsics:
        _build_extrinsics_tf_graph(
            og=og,
            graph_path=f"{robot_prim_path}/ExtrinsicsTfGraph",
            namespace=namespace,
            transforms=extrinsics,
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
    """Mirrors test_ros2_odometry.py:107-145 with namespacing added.

    Also broadcasts the ``odom_frame_id -> chassis_frame_id`` transform on
    ``<namespace>/tf`` via ROS2PublishRawTransformTree, fed by the same
    ComputeOdometry node. ROS2PublishTransformTree can only emit raw USD
    prim names (no prefix), so it can't produce the prefixed
    ``<ns>/base_link`` frame an external SLAM consumer (RTAB-Map) expects —
    this raw publisher closes that gap. Republished every playback tick
    (not `staticPublisher`) so it never ages out of the tf2 buffer.
    """
    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": graph_path, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("ComputeOdometry", "isaacsim.core.nodes.IsaacComputeOdometry"),
                ("PublishOdometry", "isaacsim.ros2.bridge.ROS2PublishOdometry"),
                ("PublishOdomTf", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"),
            ],
            keys.SET_VALUES: [
                ("ComputeOdometry.inputs:chassisPrim", [usdrt.Sdf.Path(chassis_prim_path)]),
                ("PublishOdometry.inputs:topicName", topic),
                ("PublishOdometry.inputs:nodeNamespace", namespace),
                ("PublishOdometry.inputs:odomFrameId", odom_frame_id),
                ("PublishOdometry.inputs:chassisFrameId", chassis_frame_id),
                ("PublishOdometry.inputs:publishRawVelocities", False),
                # Global /tf (NOT /<ns>/tf): ROS 2 tf2 TransformListeners
                # subscribe to the absolute /tf regardless of their own
                # namespace, so a namespaced tf topic has zero consumers. Our
                # frame_ids are already prefixed with <ns>/, so a shared global
                # /tf stays multi-robot-safe.
                ("PublishOdomTf.inputs:topicName", "tf"),
                ("PublishOdomTf.inputs:nodeNamespace", ""),
                ("PublishOdomTf.inputs:parentFrameId", odom_frame_id),
                ("PublishOdomTf.inputs:childFrameId", chassis_frame_id),
            ],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "ComputeOdometry.inputs:execIn"),
                ("ComputeOdometry.outputs:execOut", "PublishOdometry.inputs:execIn"),
                ("ComputeOdometry.outputs:execOut", "PublishOdomTf.inputs:execIn"),
                ("ComputeOdometry.outputs:position", "PublishOdometry.inputs:position"),
                ("ComputeOdometry.outputs:orientation", "PublishOdometry.inputs:orientation"),
                ("ComputeOdometry.outputs:linearVelocity", "PublishOdometry.inputs:linearVelocity"),
                ("ComputeOdometry.outputs:angularVelocity", "PublishOdometry.inputs:angularVelocity"),
                ("ComputeOdometry.outputs:position", "PublishOdomTf.inputs:translation"),
                ("ComputeOdometry.outputs:orientation", "PublishOdomTf.inputs:rotation"),
                ("ReadSimTime.outputs:simulationTime", "PublishOdometry.inputs:timeStamp"),
                ("ReadSimTime.outputs:simulationTime", "PublishOdomTf.inputs:timeStamp"),
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
    camera_info_topic: str,
    width: int,
    height: int,
    frame_id: str,
) -> None:
    """One render product feeds three ROS2 publishers (rgb, depth, camera_info)."""
    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": graph_path, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("CreateRenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                ("RGBPublish", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("DepthPublish", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("CameraInfoPublish", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
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
                ("CameraInfoPublish.inputs:topicName", camera_info_topic),
                ("CameraInfoPublish.inputs:nodeNamespace", namespace),
                ("CameraInfoPublish.inputs:frameId", frame_id),
                ("CameraInfoPublish.inputs:resetSimulationTimeOnStop", True),
            ],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "CreateRenderProduct.inputs:execIn"),
                ("CreateRenderProduct.outputs:execOut", "RGBPublish.inputs:execIn"),
                ("CreateRenderProduct.outputs:execOut", "DepthPublish.inputs:execIn"),
                ("CreateRenderProduct.outputs:execOut", "CameraInfoPublish.inputs:execIn"),
                ("CreateRenderProduct.outputs:renderProductPath", "RGBPublish.inputs:renderProductPath"),
                ("CreateRenderProduct.outputs:renderProductPath", "DepthPublish.inputs:renderProductPath"),
                ("CreateRenderProduct.outputs:renderProductPath", "CameraInfoPublish.inputs:renderProductPath"),
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


def _build_extrinsics_tf_graph(
    *,
    og,
    graph_path: str,
    namespace: str,
    transforms,
) -> None:
    """Broadcast fixed parent->child transforms on ``<namespace>/tf``.

    `transforms` is a list of ``(parent_frame_id, child_frame_id,
    ((tx,ty,tz), (qx,qy,qz,qw)))``. One ROS2PublishRawTransformTree per
    transform, all driven by a shared OnPlaybackTick + ReadSimTime so they
    re-broadcast every tick (kept dynamic rather than `staticPublisher` so
    they carry a fresh sim timestamp and never age out of the tf2 buffer
    while `use_sim_time` is on).
    """
    keys = og.Controller.Keys
    create = [
        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
        ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
    ]
    set_values = []
    connect = []
    for i, (parent_fid, child_fid, (translation, rotation)) in enumerate(transforms):
        node = f"Tf{i}"
        create.append((node, "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"))
        set_values += [
            # Global /tf — see the PublishOdomTf comment in the odometry graph.
            (f"{node}.inputs:topicName", "tf"),
            (f"{node}.inputs:nodeNamespace", ""),
            (f"{node}.inputs:parentFrameId", parent_fid),
            (f"{node}.inputs:childFrameId", child_fid),
            (f"{node}.inputs:translation", [float(v) for v in translation]),
            (f"{node}.inputs:rotation", [float(v) for v in rotation]),
        ]
        connect += [
            ("OnPlaybackTick.outputs:tick", f"{node}.inputs:execIn"),
            ("ReadSimTime.outputs:simulationTime", f"{node}.inputs:timeStamp"),
        ]
    og.Controller.edit(
        {"graph_path": graph_path, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: create,
            keys.SET_VALUES: set_values,
            keys.CONNECT: connect,
        },
    )


def _build_cmd_vel_graph(
    *,
    og,
    graph_path: str,
    namespace: str,
    topic: str,
) -> None:
    """Subscribe to `<namespace>/cmd_vel`; outputs read by the spawn loop.

    `spawn_warehouse.py` calls
    `og.Controller.attribute(f"{graph_path}/SubscribeTwist.outputs:linearVelocity").get()`
    each tick and pipes the result into `ArticulationAction(joint_velocities=...)`.
    Default outputs are zeros, so the robot is still until a Twist arrives.
    """
    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": graph_path, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("SubscribeTwist", "isaacsim.ros2.bridge.ROS2SubscribeTwist"),
            ],
            keys.SET_VALUES: [
                ("SubscribeTwist.inputs:topicName", topic),
                ("SubscribeTwist.inputs:nodeNamespace", namespace),
            ],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "SubscribeTwist.inputs:execIn"),
            ],
        },
    )


def _build_imu_graph(
    *,
    og,
    usdrt,
    graph_path: str,
    imu_prim_path: str,
    namespace: str,
    topic: str,
    frame_id: str,
) -> None:
    """IsaacReadIMU → ROS2PublishImu on `<namespace>/imu`.

    The IMU prim itself must already exist at `imu_prim_path` (authored
    in `spawn_warehouse._spawn_robot` via `IsaacSensorCreateImuSensor`).
    Publishes linear acceleration + angular velocity + fused orientation
    (the fused orientation is "richer than real" for an MPU 6050 — see
    Phase 4 docs).
    """
    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": graph_path, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("ReadIMU", "isaacsim.sensors.physics.IsaacReadIMU"),
                ("PublishImu", "isaacsim.ros2.bridge.ROS2PublishImu"),
            ],
            keys.SET_VALUES: [
                ("ReadIMU.inputs:imuPrim", [usdrt.Sdf.Path(imu_prim_path)]),
                ("ReadIMU.inputs:readGravity", True),
                ("PublishImu.inputs:topicName", topic),
                ("PublishImu.inputs:nodeNamespace", namespace),
                ("PublishImu.inputs:frameId", frame_id),
                ("PublishImu.inputs:publishLinearAcceleration", True),
                ("PublishImu.inputs:publishAngularVelocity", True),
                ("PublishImu.inputs:publishOrientation", True),
            ],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "ReadIMU.inputs:execIn"),
                ("ReadIMU.outputs:execOut", "PublishImu.inputs:execIn"),
                ("ReadIMU.outputs:linAcc", "PublishImu.inputs:linearAcceleration"),
                ("ReadIMU.outputs:angVel", "PublishImu.inputs:angularVelocity"),
                ("ReadIMU.outputs:orientation", "PublishImu.inputs:orientation"),
                ("ReadSimTime.outputs:simulationTime", "PublishImu.inputs:timeStamp"),
            ],
        },
    )
