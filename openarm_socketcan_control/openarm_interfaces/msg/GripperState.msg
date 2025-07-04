# Gripper State Message
# This message contains comprehensive gripper state information

std_msgs/Header header

# Current gripper state
float64 position           # Current position (radians)
float64 velocity           # Current velocity (rad/s)
float64 effort             # Current effort/torque (Nm)
float64 normalized_position # Normalized position (0.0 = open, 1.0 = closed)

# Physical state
float64 width              # Current gripper width (m)
float64 force              # Current gripping force (N)
float64 temperature        # Motor temperature (Celsius)

# Control state
string control_mode        # Current control mode
bool enabled               # Motor enabled state
bool is_moving            # Whether gripper is currently moving
bool is_gripping          # Whether gripper is gripping an object
bool error                # Error state
string error_message      # Error description

# Internal state
float64 timestamp         # Internal timestamp from motor controller
