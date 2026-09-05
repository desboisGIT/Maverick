# Host -> bridge position setpoint. The bridge drops it if stale (> 200 ms) or invalid.
builtin_interfaces/Time stamp   # host wall-clock time when issued
float32[3] position_ned         # metres, PX4 local frame (north, east, down)
float32 yaw                     # radians; NaN = keep current heading
