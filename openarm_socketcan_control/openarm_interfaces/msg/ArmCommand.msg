# Arm Command Message
# This message contains commands specifically for arm control

std_msgs/Header header

# Control mode
string mode           # "position", "velocity", "torque", "mit"

# Joint positions (radians)
float64[] positions

# Joint velocities (rad/s) - optional, used in velocity and MIT modes
float64[] velocities

# Joint efforts/torques (Nm) - optional, used in torque and MIT modes
float64[] efforts

# MIT control parameters - optional, used in MIT mode
float64[] kp          # Proportional gains
float64[] kd          # Derivative gains

# Safety limits - optional
float64[] position_limits_lower
float64[] position_limits_upper
float64[] velocity_limits
float64[] effort_limits
