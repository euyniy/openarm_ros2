# Gripper Command Message
# This message contains commands specifically for gripper control

std_msgs/Header header

# Control mode
string mode           # "position", "normalized", "velocity", "torque", "mit", "open", "close"

# Raw position (radians) - used in position mode
float64 position

# Normalized position (0.0 = open, 1.0 = closed) - used in normalized mode
float64 normalized_position

# Velocity (rad/s) - used in velocity mode
float64 velocity

# Effort/torque (Nm) - used in torque mode
float64 effort

# MIT control parameters - used in MIT mode
float64 kp            # Proportional gain
float64 kd            # Derivative gain

# Force/width control - advanced features
float64 target_force  # Target gripping force (N)
float64 max_width     # Maximum gripper width (m)
