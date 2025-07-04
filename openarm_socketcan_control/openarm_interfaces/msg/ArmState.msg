# Arm State Message
# This message contains comprehensive arm state information compatible with ros2-control

std_msgs/Header header

# Joint names
string[] joint_names

# Current joint states
float64[] positions      # Current positions (radians)
float64[] velocities     # Current velocities (rad/s)
float64[] efforts        # Current efforts/torques (Nm)

# Additional state information
float64[] temperatures   # Motor temperatures (Celsius)
bool[] enabled          # Motor enabled states
bool[] errors           # Motor error states
string[] error_messages # Error descriptions

# Control state
string control_mode     # Current control mode
bool is_moving         # Whether arm is currently moving
float64 timestamp      # Internal timestamp from motor controller
