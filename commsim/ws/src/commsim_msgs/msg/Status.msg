# Bundled heartbeat (V2+). Best-effort, newest wins. Path and obstacles are
# empty except right after a replan / when something new was seen, which is
# what keeps the typical size near the low end of ~100-300 B.
FleetHeader header
Point2 pose
float32 yaw
Point2 velocity
uint8 battery_pct
OwnedTask[] owned
Point2[<=10] path
Obstacle[] new_obstacles
