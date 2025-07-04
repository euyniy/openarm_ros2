# MIT Control Command for a single joint
# This message contains MIT control parameters for position, velocity, and torque control

uint32 joint_index    # Joint index (0-based)
float64 kp           # Proportional gain
float64 kd           # Derivative gain
float64 position     # Target position (radians)
float64 velocity     # Target velocity (rad/s)
float64 torque       # Feed-forward torque (Nm)
