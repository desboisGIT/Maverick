# Host -> bridge vehicle command. The bridge drops it if stale (> 500 ms).
uint8 ARM = 1
uint8 DISARM = 2
uint8 OFFBOARD = 3
uint8 LAND = 4
uint8 HOLD = 5

builtin_interfaces/Time stamp   # host wall-clock time when issued
uint8 command
