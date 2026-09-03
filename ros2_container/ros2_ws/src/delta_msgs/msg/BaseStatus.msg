# delta_msgs/msg/BaseStatus.msg
#
# Mobile base status, derived from the Arduino's telemetry frames.
# Published by mobile_base_bridge_node on /base/status; relayed to the
# frontend by frontend_bridge_node as a "base_status" websocket message.

bool moving          # true if either linear or angular actual speed is nonzero
bool estop_active    # true if the physical E-stop is currently sensed as tripped
uint8 laser_pct      # 0-100, current laser power
int8 linear_pct      # -100..100, current ramped actual linear speed (% of configured max)
int8 angular_pct     # -100..100, current ramped actual angular speed (% of configured max)