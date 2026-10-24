# Common header on every fleet message (14 B of CDR payload before padding).
# Named FleetHeader, not Header, so it never shadows std_msgs/Header.
uint8 version
uint8 robot_id
uint32 seq
builtin_interfaces/Time stamp
