# OpenArm Bimanual Bringup Package

This package provides launch files and configuration for bringing up the dual-arm OpenArm robotic system with real hardware. It manages coordinated control of both arms, shared hardware interfaces, and bimanual controller orchestration.

## 📦 Package Contents

### Directories

- **launch/**: Launch files for bimanual system startup and testing
- **config/**: Configuration files for dual-arm controllers and hardware

### Key Files

- **launch/bimanual.launch.py**: Main launch file for complete bimanual system
- **launch/left_arm.launch.py**: Launch left arm independently
- **launch/right_arm.launch.py**: Launch right arm independently
- **launch/depth_camera.launch.py**: RealSense camera for perception
- **config/bimanual_controllers.yaml**: Dual-arm controller configuration
- **config/bimanual_ros2_control.yaml**: Hardware interface for both arms

## 🚀 Usage

### Complete Bimanual System Launch

Launch the full dual-arm OpenArm system:

```bash
ros2 launch openarm_bimanual_bringup bimanual.launch.py
```

This command starts:
1. **Dual Hardware Interface**: Connects to both arms via shared CAN bus
2. **Controller Manager**: Manages controllers for both arms
3. **Robot State Publisher**: Publishes bimanual robot model and transforms
4. **Dual Controllers**: Independent and coordinated control for both arms
5. **RealSense Camera**: Head-mounted depth perception (optional)

### Individual Arm Control

Control arms independently for testing or maintenance:

```bash
# Left arm only
ros2 launch openarm_bimanual_bringup left_arm.launch.py

# Right arm only  
ros2 launch openarm_bimanual_bringup right_arm.launch.py
```

### Perception System

Start the RealSense camera for 3D perception and OctoMap:

```bash
ros2 launch openarm_bimanual_bringup depth_camera.launch.py
```

## 🎛️ ROS2 Controllers for Bimanual System

### Available Controllers

The bimanual system provides controllers for both independent and coordinated control:

#### 1. Independent Arm Controllers

**Left Arm Controllers**:
- **left_arm_position_controller**: Direct position control for left arm
- **left_arm_trajectory_controller**: Trajectory following for left arm

```bash
# Example: Control left arm positions
ros2 topic pub /left_arm_position_controller/commands std_msgs/msg/Float64MultiArray "data: [0.0, 0.5, -0.5, 0.0, 0.0, 0.0]"

# Example: Send left arm trajectory
ros2 action send_goal /left_arm_trajectory_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "
trajectory:
  joint_names: ['left_joint_1', 'left_joint_2', 'left_joint_3', 'left_joint_4', 'left_joint_5', 'left_joint_6']
  points:
  - positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    time_from_start: {sec: 2}
  - positions: [0.5, 0.5, 0.5, 0.5, 0.5, 0.5]
    time_from_start: {sec: 4}
"
```

**Right Arm Controllers**:
- **right_arm_position_controller**: Direct position control for right arm
- **right_arm_trajectory_controller**: Trajectory following for right arm

```bash
# Example: Control right arm positions
ros2 topic pub /right_arm_position_controller/commands std_msgs/msg/Float64MultiArray "data: [0.0, -0.5, 0.5, 0.0, 0.0, 0.0]"

# Example: Send right arm trajectory
ros2 action send_goal /right_arm_trajectory_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "
trajectory:
  joint_names: ['right_joint_1', 'right_joint_2', 'right_joint_3', 'right_joint_4', 'right_joint_5', 'right_joint_6']
  points:
  - positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    time_from_start: {sec: 2}
  - positions: [-0.5, -0.5, -0.5, -0.5, -0.5, -0.5]
    time_from_start: {sec: 4}
"
```

#### 2. Coordinated Bimanual Controllers

**Bimanual Group Controller**:
- **bimanual_trajectory_controller**: Coordinated motion for both arms

```bash
# Example: Coordinated bimanual trajectory
ros2 action send_goal /bimanual_trajectory_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "
trajectory:
  joint_names: ['left_joint_1', 'left_joint_2', 'left_joint_3', 'left_joint_4', 'left_joint_5', 'left_joint_6',
                'right_joint_1', 'right_joint_2', 'right_joint_3', 'right_joint_4', 'right_joint_5', 'right_joint_6']
  points:
  - positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    time_from_start: {sec: 2}
  - positions: [0.5, 0.5, 0.5, 0.5, 0.5, 0.5, -0.5, -0.5, -0.5, -0.5, -0.5, -0.5]
    time_from_start: {sec: 4}
"
```

#### 3. Joint State Broadcasting

- **joint_state_broadcaster**: Publishes states for all 12 joints (6 per arm)

```bash
# Monitor all joint states
ros2 topic echo /joint_states
```

### Advanced Controller Operations

#### Switching Between Control Modes

```bash
# Switch to independent control
ros2 control switch_controllers --activate left_arm_trajectory_controller right_arm_trajectory_controller --deactivate bimanual_trajectory_controller

# Switch to coordinated control
ros2 control switch_controllers --activate bimanual_trajectory_controller --deactivate left_arm_trajectory_controller right_arm_trajectory_controller
```

#### Controller Status Monitoring

```bash
# List all controllers
ros2 control list_controllers

# Check specific controller status
ros2 control list_controllers | grep -E "(left_arm|right_arm|bimanual)"

# Get detailed controller info
ros2 control describe_controller left_arm_trajectory_controller
```

## ⚙️ System Architecture

### Hardware Configuration

The bimanual system uses a shared CAN bus with different ID ranges:

- **Left Arm**: CAN IDs 0x01-0x06 (plus optional gripper 0x07)
- **Right Arm**: CAN IDs 0x11-0x16 (plus optional gripper 0x17)
- **Shared CAN Bus**: Both arms communicate on same CAN interface
- **RealSense Camera**: Head-mounted depth camera for perception

### Controller Setup

The bimanual system includes multiple controller configurations:

```yaml
# Example bimanual controller configuration
controller_manager:
  ros__parameters:
    update_rate: 100
    
    # Hardware interfaces
    left_arm_hardware:
      type: openarm_hardware/OpenArmHW
    right_arm_hardware:
      type: openarm_hardware/OpenArmHW
    
    # Controllers
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster
      
    left_arm_trajectory_controller:
      type: joint_trajectory_controller/JointTrajectoryController
      
    right_arm_trajectory_controller:
      type: joint_trajectory_controller/JointTrajectoryController
      
    bimanual_trajectory_controller:
      type: joint_trajectory_controller/JointTrajectoryController

# Hardware parameters
left_arm_hardware:
  ros__parameters:
    can_device: "can0"
    prefix: "left_"
    motor_id_offset: 0x00

right_arm_hardware:
  ros__parameters:
    can_device: "can0"  # Shared CAN bus
    prefix: "right_"
    motor_id_offset: 0x10
```

### Namespace Configuration

- **Left Arm**: `/left_arm/` namespace for controllers and topics
- **Right Arm**: `/right_arm/` namespace for controllers and topics
- **Global**: Shared transforms, camera data, and joint states

## 🔧 CAN Bus Setup

### Shared CAN Interface

Both arms share a single CAN interface but use different ID ranges:

```bash
# Initialize shared CAN bus
cd ~/ros2_ws/src/openarm_ros2/openarm_bringup/utils
./init_can.sh /dev/ttyACM0 can0
```

### Motor ID Configuration

**Left Arm Motor IDs**:
- Joint 1: 0x01 (Master: 0x11)
- Joint 2: 0x02 (Master: 0x12)
- Joint 3: 0x03 (Master: 0x13)
- Joint 4: 0x04 (Master: 0x14)
- Joint 5: 0x05 (Master: 0x15)
- Joint 6: 0x06 (Master: 0x16)
- Gripper: 0x07 (Master: 0x17) [Optional]

**Right Arm Motor IDs**:
- Joint 1: 0x11 (Master: 0x21)
- Joint 2: 0x12 (Master: 0x22)
- Joint 3: 0x13 (Master: 0x23)
- Joint 4: 0x14 (Master: 0x24)
- Joint 5: 0x15 (Master: 0x25)
- Joint 6: 0x16 (Master: 0x26)
- Gripper: 0x17 (Master: 0x27) [Optional]

## 🎯 Bimanual Applications

This setup enables advanced bimanual manipulation:

### Coordinated Manipulation

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class BimanualController(Node):
    def __init__(self):
        super().__init__('bimanual_controller')
        
        # Action clients for both arms
        self.left_client = ActionClient(
            self, FollowJointTrajectory, '/left_arm_trajectory_controller/follow_joint_trajectory')
        self.right_client = ActionClient(
            self, FollowJointTrajectory, '/right_arm_trajectory_controller/follow_joint_trajectory')
        self.bimanual_client = ActionClient(
            self, FollowJointTrajectory, '/bimanual_trajectory_controller/follow_joint_trajectory')
    
    def coordinated_motion(self):
        """Execute coordinated bimanual motion"""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = [
            'left_joint_1', 'left_joint_2', 'left_joint_3', 'left_joint_4', 'left_joint_5', 'left_joint_6',
            'right_joint_1', 'right_joint_2', 'right_joint_3', 'right_joint_4', 'right_joint_5', 'right_joint_6'
        ]
        
        # Define coordinated trajectory points
        point1 = JointTrajectoryPoint()
        point1.positions = [0.0] * 12
        point1.time_from_start.sec = 2
        
        point2 = JointTrajectoryPoint()
        point2.positions = [0.5, 0.5, 0.5, 0.5, 0.5, 0.5, -0.5, -0.5, -0.5, -0.5, -0.5, -0.5]
        point2.time_from_start.sec = 4
        
        goal.trajectory.points = [point1, point2]
        
        self.bimanual_client.send_goal_async(goal)
    
    def handover_motion(self):
        """Execute object handover between arms"""
        # Left arm picks up object
        left_goal = FollowJointTrajectory.Goal()
        left_goal.trajectory.joint_names = ['left_joint_1', 'left_joint_2', 'left_joint_3', 'left_joint_4', 'left_joint_5', 'left_joint_6']
        
        # Define pickup trajectory
        pickup_point = JointTrajectoryPoint()
        pickup_point.positions = [0.3, 0.5, -0.8, 0.0, 0.5, 0.0]  # Pickup pose
        pickup_point.time_from_start.sec = 3
        
        handover_point = JointTrajectoryPoint()
        handover_point.positions = [0.0, 0.3, -0.3, 0.0, 0.0, 0.0]  # Handover pose
        handover_point.time_from_start.sec = 6
        
        left_goal.trajectory.points = [pickup_point, handover_point]
        
        # Right arm receives object
        right_goal = FollowJointTrajectory.Goal()
        right_goal.trajectory.joint_names = ['right_joint_1', 'right_joint_2', 'right_joint_3', 'right_joint_4', 'right_joint_5', 'right_joint_6']
        
        receive_point = JointTrajectoryPoint()
        receive_point.positions = [0.0, -0.3, 0.3, 0.0, 0.0, 0.0]  # Receive pose
        receive_point.time_from_start.sec = 6
        
        right_goal.trajectory.points = [receive_point]
        
        # Execute both trajectories
        self.left_client.send_goal_async(left_goal)
        self.right_client.send_goal_async(right_goal)

def main():
    rclpy.init()
    controller = BimanualController()
    
    # Example usage
    controller.coordinated_motion()
    # controller.handover_motion()
    
    rclpy.spin(controller)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Object Handover

- **Pickup Phase**: Left arm approaches and grasps object
- **Transfer Phase**: Both arms move to handover position
- **Release Phase**: Right arm receives object, left arm releases

### Assembly Tasks

- **Coordinated Assembly**: Both arms work on same component
- **Part Holding**: One arm holds while other manipulates
- **Dual Tool Operation**: Each arm uses different tools

## 🛠️ Prerequisites

### Hardware Setup

1. **Dual CAN Interface**: Single CAN bus serving both arms
2. **Motor Controllers**: 12 DM series motors (6 per arm) with unique IDs
3. **Power Supply**: Sufficient current for dual motor operation
4. **Emergency Stop**: Single e-stop affects both arms
5. **RealSense Camera**: D435 mounted on head for perception

### Software Dependencies

```bash
# Install bimanual-specific packages
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers
sudo apt install ros-humble-controller-manager ros-humble-hardware-interface
sudo apt install ros-humble-realsense2-camera ros-humble-realsense2-description
sudo apt install ros-humble-octomap-server ros-humble-moveit-servo
```

## 🚨 Safety Considerations

### Collision Avoidance

- **Arm Separation**: Maintain safe distance between arms during motion
- **Workspace Limits**: Configure non-overlapping safe zones when needed
- **Emergency Stop**: Single e-stop immediately stops both arms
- **Collision Detection**: Monitor for arm-to-arm contact in real-time

### Operational Safety

- **Test individually first**: Verify each arm works before bimanual operation
- **Verify CAN ID separation**: Ensure no conflicts between arm motor IDs
- **Monitor power consumption**: Dual motors require more power
- **Ensure adequate cooling**: Extended dual operation generates more heat

### Emergency Procedures

1. **Emergency Stop**: Press physical e-stop (affects both arms)
2. **Software Emergency Stop**: 
   ```bash
   ros2 service call /controller_manager/emergency_stop std_srvs/srv/Trigger
   ```
3. **Individual Arm Stop**:
   ```bash
   ros2 control set_controller_state left_arm_trajectory_controller inactive
   ros2 control set_controller_state right_arm_trajectory_controller inactive
   ```

## 🐛 Troubleshooting

### Common Issues

#### 1. CAN ID Conflicts
**Symptoms**: Erratic behavior, motors responding to wrong commands

**Solutions**:
```bash
# Monitor CAN traffic for conflicts
candump can0 | grep -E "(01|02|03|04|05|06|11|12|13|14|15|16)"

# Verify motor ID configuration
ros2 param get /left_arm_hardware motor_ids
ros2 param get /right_arm_hardware motor_ids

# Check for duplicate IDs in configuration files
```

#### 2. Controller Conflicts
**Symptoms**: Controllers fail to start, resource conflicts

**Solutions**:
```bash
# Check controller resource allocation
ros2 control list_hardware_interfaces

# Verify controller configuration
ros2 param list /controller_manager | grep -E "(left_arm|right_arm|bimanual)"

# Restart controller manager
ros2 lifecycle set /controller_manager configure
ros2 lifecycle set /controller_manager activate
```

#### 3. Synchronization Issues
**Symptoms**: Arms move out of sync, timing problems

**Solutions**:
- Check system clock synchronization
- Verify control loop timing consistency
- Monitor communication latency between arms
- Adjust trajectory timing tolerances

#### 4. Perception Integration Issues
**Symptoms**: OctoMap not updating, camera data missing

**Solutions**:
```bash
# Check camera status
ros2 topic list | grep camera
ros2 topic echo /camera/depth/color/points --once

# Verify OctoMap updates
ros2 topic echo /octomap_binary --once

# Check transform tree
ros2 run tf2_tools view_frames.py
```

### Debug Commands

```bash
# Monitor bimanual system status
ros2 topic echo /joint_states | grep -E "(left_|right_)"

# Check controller performance
ros2 topic echo /left_arm_trajectory_controller/state
ros2 topic echo /right_arm_trajectory_controller/state

# Monitor CAN bus load
canbusload can0@1000000

# Check hardware interface status
ros2 service call /controller_manager/list_hardware_interfaces controller_manager_msgs/srv/ListHardwareInterfaces
```

## 📊 Performance Monitoring

### Key Metrics for Bimanual System

- **Dual Control Loop Frequency**: Both arms should maintain ~100Hz
- **CAN Bus Utilization**: Monitor for overload with 12 motors
- **Synchronization Accuracy**: Measure timing between coordinated motions
- **Inter-Arm Communication Latency**: Check coordination delays

### Monitoring Tools

```bash
# Real-time bimanual plotting
ros2 run rqt_plot rqt_plot /joint_states/position[0] /joint_states/position[6]

# System resource monitoring
htop  # Check CPU usage for dual control
iotop  # Monitor I/O for dual hardware interfaces

# CAN bus monitoring with dual arms
canbusload can0@1000000  # Should show higher utilization
candump can0 | wc -l     # Count messages per second
```

## 🔗 Integration Examples

### With MoveIt2 Bimanual Planning

```bash
# Terminal 1: Start bimanual hardware
ros2 launch openarm_bimanual_bringup bimanual.launch.py

# Terminal 2: Start bimanual MoveIt
ros2 launch openarm_bimanual_moveit_config move_group.launch.py

# Terminal 3: Start perception
ros2 launch openarm_bimanual_bringup depth_camera.launch.py

# Terminal 4: Start RViz
ros2 launch openarm_bimanual_moveit_config moveit_rviz.launch.py
```

### Custom Bimanual Application Template

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import numpy as np

class BimanualApplication(Node):
    def __init__(self):
        super().__init__('bimanual_application')
        
        # Subscribers
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_callback, 10)
        
        # Publishers for direct control
        self.left_cmd_pub = self.create_publisher(
            Float64MultiArray, '/left_arm_position_controller/commands', 10)
        self.right_cmd_pub = self.create_publisher(
            Float64MultiArray, '/right_arm_position_controller/commands', 10)
        
        # State variables
        self.left_positions = np.zeros(6)
        self.right_positions = np.zeros(6)
        
        # Control timer
        self.timer = self.create_timer(0.01, self.control_loop)  # 100Hz
    
    def joint_callback(self, msg):
        """Process joint states for both arms"""
        if len(msg.position) >= 12:
            self.left_positions = np.array(msg.position[:6])
            self.right_positions = np.array(msg.position[6:12])
    
    def control_loop(self):
        """Main control loop for bimanual coordination"""
        # Example: Mirror motion between arms
        left_cmd = Float64MultiArray()
        right_cmd = Float64MultiArray()
        
        # Generate coordinated commands
        t = self.get_clock().now().nanoseconds * 1e-9
        amplitude = 0.3
        frequency = 0.1
        
        # Sinusoidal motion for demonstration
        left_target = amplitude * np.sin(2 * np.pi * frequency * t)
        right_target = -amplitude * np.sin(2 * np.pi * frequency * t)  # Mirror
        
        left_cmd.data = [left_target, 0.0, 0.0, 0.0, 0.0, 0.0]
        right_cmd.data = [right_target, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        self.left_cmd_pub.publish(left_cmd)
        self.right_cmd_pub.publish(right_cmd)

def main():
    rclpy.init()
    app = BimanualApplication()
    rclpy.spin(app)
    rclpy.shutdown()

if __name__ == '__main__':
    main() 