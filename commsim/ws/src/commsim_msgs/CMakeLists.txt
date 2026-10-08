cmake_minimum_required(VERSION 3.8)
project(commsim_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FleetHeader.msg"
  "msg/Point2.msg"
  "msg/OwnedTask.msg"
  "msg/Obstacle.msg"
  "msg/TaskAnnounce.msg"
  "msg/Bid.msg"
  "msg/Claim.msg"
  "msg/TaskEvent.msg"
  "msg/Status.msg"
  "msg/ObstacleEvent.msg"
  "msg/MapUpdate.msg"
  "msg/MapResync.msg"
  "msg/Emergency.msg"
  "msg/LinkHealth.msg"
  "msg/LeaseBeacon.msg"
  DEPENDENCIES builtin_interfaces
)

ament_package()
