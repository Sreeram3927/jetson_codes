# Unified command envelope consumed by manipulator_bridge, produced by command_arbiter.
uint8 CMD_STOP=0
uint8 CMD_MOVE_ABSOLUTE=1
uint8 CMD_JOG_RELATIVE=2
uint8 CMD_MOVE_COORDINATE=3
uint8 MODE_LASER=4

uint8 mode
float32[3] joint_target       # MODE_JOINT: target angles per joint (degrees)
float32[3] jog_velocity       # MODE_JOG: per-axis jog velocity
string motor_id                # motorId to target jog commands
geometry_msgs/Point coordinate_target   # MODE_COORDINATE: target XYZ, manipulator frame
float32 feed_rate             # MODE_COORDINATE: feed rate
bool laser_enable             # MODE_LASER
uint8 laser_duty              # MODE_LASER: 0-255
