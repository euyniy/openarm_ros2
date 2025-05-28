# OpenArm Bringup Package

This package provides launch files, configuration files, and utilities for bringing up the single OpenArm robotic system with real hardware. It manages the complete system startup including hardware interface, ROS2 controllers, and robot state publishing.

## 📦 Package Contents

### Directories

- **launch/**: Launch files for system startup and testing
- **config/**: Configuration files for controllers and hardware parameters
- **utils/**: Utility scripts for system setup and maintenance

### Key Files

- **launch/openarm.launch.py**: Main launch file for complete system startup
- **launch/test_forward_position_controller.launch.py**: Test forward position control
- **launch/test_joint_trajectory_controller.launch.py**: Test trajectory following
- **config/openarm_controllers.yaml**: Controller configuration and parameters
- **config/ros2_control.yaml**: Hardware interface configuration
- **utils/init_can.sh**: CAN bus initialization script

## 🚀 Usage

### Complete System Launch

Launch the full OpenArm system with hardware interface and controllers:

```bash
ros2 launch openarm_bringup openarm.launch.py
```

This command starts:
1. **Hardware Interface**: Connects to physical motors via CAN bus
2. **Controller Manager**: Manages ros2_control controllers
3. **Robot State Publisher**: Publishes robot model and transforms
4. **Joint Controllers**: Position and trajectory controllers for motion

### Test Controllers

Test individual controllers to verify system functionality:

```bash
# Test position controller
ros2 launch openarm_bringup test_forward_position_controller.launch.py

# Test trajectory controller  
ros2 launch openarm_bringup test_joint_trajectory_controller.launch.py
```

### CAN Bus Setup

**IMPORTANT**: Initialize CAN interface before launching the system:

```bash
# Navigate to utils directory
cd ~/ros2_ws/src/openarm_ros2/openarm_bringup/utils

# Initialize CAN bus (replace with your device)
./init_can.sh /dev/ttyACM0 can0
```

## 🎛️ ROS2 Controllers

### Available Controllers

The system provides several controllers for different control modes:

#### 1. Forward Position Controller
- **Type**: `forward_command_controller/ForwardCommandController`
- **Interface**: Position commands
- **Use Case**: Direct joint position control
- **Topic**: `/forward_position_controller/commands`

```bash
# Example: Send position commands
ros2 topic pub /forward_position_controller/commands std_msgs/msg/Float64MultiArray "data: [0.0, 0.5, -0.5, 0.0, 0.0, 0.0]"
```

#### 2. Joint Trajectory Controller
- **Type**: `joint_trajectory_controller/JointTrajectoryController`
- **Interface**: Trajectory following
- **Use Case**: Smooth trajectory execution
- **Action**: `/joint_trajectory_controller/follow_joint_trajectory`

```bash
# Example: Send trajectory goal
ros2 action send_goal /joint_trajectory_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "
trajectory:
  joint_names: ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
  points:
  - positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    time_from_start: {sec: 2}
  - positions: [0.5, 0.5, 0.5, 0.5, 0.5, 0.5]
    time_from_start: {sec: 4}
"
```

#### 3. Joint State Broadcaster
- **Type**: `joint_state_broadcaster/JointStateBroadcaster`
- **Interface**: State publishing
- **Use Case**: Publishes current joint states
- **Topic**: `/joint_states`

### Controller Management

#### List Active Controllers
```bash
ros2 control list_controllers
```

#### Load Controller
```bash
ros2 control load_controller <controller_name>
```

#### Start/Stop Controllers
```bash
# Start controller
ros2 control set_controller_state <controller_name> active

# Stop controller
ros2 control set_controller_state <controller_name> inactive
```

#### Switch Controllers
```bash
ros2 control switch_controllers --activate <controller1> --deactivate <controller2>
```

## ⚙️ Configuration

### Hardware Configuration

Edit `config/ros2_control.yaml` to configure:

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100  # Control loop frequency (Hz)
    
    # Hardware interface
    openarm_hardware:
      type: openarm_hardware/OpenArmHW
      
    # Controllers
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster
      
    forward_position_controller:
      type: forward_command_controller/ForwardCommandController
      
    joint_trajectory_controller:
      type: joint_trajectory_controller/JointTrajectoryController

# Hardware parameters
openarm_hardware:
  ros__parameters:
    can_device: "can0"
    prefix: ""
    disable_torque: false
```

### Controller Configuration

Edit `config/openarm_controllers.yaml` for controller-specific settings:

```yaml
forward_position_controller:
  ros__parameters:
    joints:
      - joint_1
      - joint_2
      - joint_3
      - joint_4
      - joint_5
      - joint_6
    interface_name: position

joint_trajectory_controller:
  ros__parameters:
    joints:
      - joint_1
      - joint_2
      - joint_3
      - joint_4
      - joint_5
      - joint_6
    
    command_interfaces:
      - position
      
    state_interfaces:
      - position
      - velocity
      
    state_publish_rate: 50.0
    action_monitor_rate: 20.0
    
    allow_partial_joints_goal: false
    constraints:
      stopped_velocity_tolerance: 0.01
      goal_time: 0.6
      joint_1:
        trajectory: 0.1
        goal: 0.05
      joint_2:
        trajectory: 0.1
        goal: 0.05
      # ... (similar for other joints)
```

## 🔧 System Components

### Hardware Interface

The hardware interface (`openarm_hardware`) provides:

- **State Interfaces**: Position, velocity, effort for each joint
- **Command Interfaces**: Position, velocity, effort commands
- **CAN Communication**: Real-time motor control via CAN bus
- **Safety Features**: Joint limits, emergency stop, torque limiting

### Robot State Publisher

Publishes robot description and transforms:
- **URDF**: Robot model from `openarm_description`
- **TF**: Transform tree for all robot links
- **Joint States**: Current joint positions and velocities

## 🛠️ Prerequisites

### Hardware Setup

1. **CAN Interface**: USB-to-CAN adapter or built-in CAN controller
2. **Motor Controllers**: DM series servo motors with CAN communication
3. **Power Supply**: Appropriate voltage and current for motors (typically 24V)
4. **Emergency Stop**: Physical e-stop button (highly recommended)
5. **Proper Grounding**: Ensure all components share common ground

### Software Dependencies

```bash
# Install required packages
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers
sudo apt install ros-humble-controller-manager ros-humble-hardware-interface
sudo apt install can-utils  # For CAN bus utilities
```

### Permissions Setup

```bash
# Add user to dialout group for CAN device access
sudo usermod -a -G dialout $USER

# Logout and login again, or run:
newgrp dialout
```

## 🔍 Utilities

### init_can.sh

Initializes CAN bus interface with proper settings:

```bash
./init_can.sh <device> <can_interface>
```

**Parameters**:
- `<device>`: USB device path (e.g., `/dev/ttyACM0`)
- `<can_interface>`: CAN interface name (e.g., `can0`)

**Examples**:
```bash
./init_can.sh /dev/ttyACM0 can0
./init_can.sh /dev/ttyACM1 can1
```

**What it does**:
1. Sets CAN bitrate to 1Mbps
2. Enables CAN-FD mode
3. Brings up the CAN interface
4. Configures proper timing parameters

### Manual CAN Setup

If the script doesn't work, manually configure CAN:

```bash
# Set CAN bitrate and enable FD mode
sudo ip link set can0 type can bitrate 1000000 dbitrate 5000000 fd on

# Bring up the interface
sudo ip link set up can0

# Verify status
ip link show can0
```

## 🚨 Safety Considerations

### Pre-Operation Checklist

- [ ] Emergency stop button is accessible and functional
- [ ] All motor connections are secure
- [ ] CAN bus termination is properly configured
- [ ] Power supply voltage matches motor specifications
- [ ] Workspace is clear of obstacles and people
- [ ] Joint limits are properly configured in URDF

### Operational Safety

- **Always test in simulation first** before using real hardware
- **Start with low gains** and gradually increase controller parameters
- **Monitor motor temperatures** during extended operation
- **Keep emergency stop within reach** at all times
- **Check joint limits** before commanding large motions
- **Verify CAN communication** before enabling motors

### Emergency Procedures

1. **Emergency Stop**: Press physical e-stop button
2. **Software Stop**: `Ctrl+C` in launch terminal
3. **Controller Stop**: 
   ```bash
   ros2 control set_controller_state joint_trajectory_controller inactive
   ```
4. **Hardware Disable**: 
   ```bash
   ros2 service call /controller_manager/set_hardware_component_state controller_manager_msgs/srv/SetHardwareComponentState "
   name: 'openarm_hardware'
   target_state: {id: 1, label: 'inactive'}
   "
   ```

## 🐛 Troubleshooting

### Common Issues

#### 1. CAN Bus Not Found
**Symptoms**: `Failed to open CAN device` error

**Solutions**:
```bash
# Check device connection
ls /dev/ttyACM*

# Verify permissions
groups $USER  # Should include 'dialout'

# Check CAN interface status
ip link show can0

# Reinitialize CAN
sudo ip link set down can0
./init_can.sh /dev/ttyACM0 can0
```

#### 2. Controller Startup Failures
**Symptoms**: Controllers fail to load or activate

**Solutions**:
```bash
# Check hardware interface status
ros2 control list_hardware_interfaces

# Verify controller configuration
ros2 param list /controller_manager

# Check for conflicting controllers
ros2 control list_controllers

# Restart controller manager
ros2 lifecycle set /controller_manager configure
ros2 lifecycle set /controller_manager activate
```

#### 3. Motor Communication Errors
**Symptoms**: No response from motors, CAN errors

**Solutions**:
```bash
# Monitor CAN traffic
candump can0

# Check for CAN errors
ip -details link show can0

# Verify motor IDs are unique
# Check motor power and connections
# Ensure proper CAN bus termination
```

#### 4. Motion Issues
**Symptoms**: Jerky motion, position errors, instability

**Solutions**:
- Check joint limits in URDF match physical robot
- Verify controller gains in configuration files
- Monitor for CAN bus overload (reduce update rate if needed)
- Check for mechanical binding or excessive friction

### Debug Commands

```bash
# Monitor system status
ros2 topic echo /joint_states
ros2 topic echo /diagnostics

# Check controller performance
ros2 topic echo /joint_trajectory_controller/state

# Monitor CAN bus
candump can0 | grep -E "(01|02|03|04|05|06)"  # Filter by motor IDs

# Hardware interface diagnostics
ros2 service call /controller_manager/list_hardware_interfaces controller_manager_msgs/srv/ListHardwareInterfaces
```

### Log Analysis

```bash
# View controller manager logs
ros2 log view controller_manager

# Check hardware interface logs
grep -i "openarm" ~/.ros/log/latest/controller_manager/stdout.log

# Monitor real-time logs
ros2 run rqt_console rqt_console
```

## 📊 Performance Monitoring

### Key Metrics

- **Control Loop Frequency**: Should maintain ~100Hz
- **CAN Bus Utilization**: Monitor with `canbusload can0`
- **Joint Position Accuracy**: Compare commanded vs actual positions
- **Communication Latency**: Check for delayed responses

### Monitoring Tools

```bash
# Real-time plotting
ros2 run rqt_plot rqt_plot /joint_states/position[0]

# System monitor
htop  # Check CPU usage
iotop  # Check I/O usage

# CAN bus monitoring
canbusload can0@1000000  # Monitor bus load
```

## 🔗 Integration with Other Packages

### With MoveIt2

```bash
# Terminal 1: Start hardware
ros2 launch openarm_bringup openarm.launch.py

# Terminal 2: Start MoveIt
ros2 launch openarm_moveit_config move_group.launch.py

# Terminal 3: Start RViz
ros2 launch openarm_moveit_config moveit_rviz.launch.py
```

### With Simulation

```bash
# Use simulation instead of real hardware
ros2 launch openarm_bringup openarm.launch.py use_sim_time:=true fake_hardware:=true
```

### Custom Applications

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

class OpenArmController(Node):
    def __init__(self):
        super().__init__('openarm_controller')
        
        # Subscribe to joint states
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_callback, 10)
        
        # Publish position commands
        self.cmd_pub = self.create_publisher(
            Float64MultiArray, '/forward_position_controller/commands', 10)
    
    def joint_callback(self, msg):
        # Process current joint states
        current_positions = msg.position
        
        # Generate new commands
        cmd = Float64MultiArray()
        cmd.data = [0.0, 0.5, -0.5, 0.0, 0.0, 0.0]  # Example positions
        
        self.cmd_pub.publish(cmd)

def main():
    rclpy.init()
    controller = OpenArmController()
    rclpy.spin(controller)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
``` 