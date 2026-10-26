# Bridge -> host diagnostics, published at 1 Hz.
builtin_interfaces/Time stamp
uint32 setpoints_forwarded
uint32 setpoints_dropped_stale    # older than 200 ms or stamped > 50 ms in the future
uint32 setpoints_dropped_invalid  # NaN/inf position
uint32 commands_forwarded
uint32 commands_dropped_stale
float32 last_setpoint_age_ms      # age when the last setpoint arrived; NaN if none yet
uint32 vision_forwarded            # visual odometry samples handed to PX4
uint32 vision_dropped_stale        # older than 200 ms or stamped > 50 ms in the future
uint32 vision_dropped_invalid      # NaN/inf or malformed
bool ev_pos_fused               # PX4's EKF fuses the external-vision position (estimator_status_flags.cs_ev_pos)
