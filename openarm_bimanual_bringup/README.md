# OpenArm Bimanual Bringup Package

This package provides launch files and configuration for bringing up the dual-arm OpenArm robotic system with real hardware.

## Package Contents

### Directories

- **launch/**: Launch files for bimanual system startup
- **config/**: Configuration files for dual-arm controllers and hardware

### Key Features

- **Dual Arm Control**: Simultaneous control of left and right arms
- **Coordinated Motion**: Synchronized bimanual manipulation
- **Independent Controllers**: Separate control for each arm
- **Shared Hardware Interface**: Unified CAN bus communication

## Usage

### Complete Bimanual System Launch

Launch the full dual-arm OpenArm system:

```bash
ros2 launch openarm_bimanual_bringup bimanual.launch.py
```

### Individual Arm Control

Control arms independently:

```bash
# Left arm only
ros2 launch openarm_bimanual_bringup left_arm.launch.py

# Right arm only  
ros2 launch openarm_bimanual_bringup right_arm.launch.py
```

## System Architecture

### Hardware Configuration

- **Left Arm**: CAN IDs 0x01-0x06 (plus optional gripper 0x07)
- **Right Arm**: CAN IDs 0x11-0x16 (plus optional gripper 0x17)
- **Shared CAN Bus**: Both arms communicate on same CAN interface
- **RealSense Camera**: Head-mounted depth camera for perception

### Controller Setup

The bimanual system includes:

1. **Left Arm Controllers**:
   - `left_arm_position_controller`
   - `left_arm_trajectory_controller`

2. **Right Arm Controllers**:
   - `right_arm_position_controller`
   - `right_arm_trajectory_controller`

3. **Shared Components**:
   - Hardware interface manager
   - Robot state publisher
   - Camera drivers

## Configuration

### CAN Bus Setup

Both arms share a single CAN interface but use different ID ranges:

```bash
# Initialize shared CAN bus
cd openarm_bringup/utils
./init_can.sh /dev/ttyACM0 can0
```

### Namespace Configuration

- **Left Arm**: `/left_arm/` namespace
- **Right Arm**: `/right_arm/` namespace
- **Global**: Shared transforms and camera data

## Bimanual Applications

This setup enables:

- **Coordinated Manipulation**: Both arms working together
- **Object Handover**: Passing objects between arms
- **Assembly Tasks**: Complex multi-handed operations
- **Human-Robot Interaction**: Natural bimanual behaviors

## Dependencies

- `openarm_bringup`: Single arm launch infrastructure
- `openarm_bimanual_description`: Dual arm robot model
- `openarm_hardware`: Shared hardware interface
- `controller_manager`: Multi-controller management
- `realsense2_camera`: Camera driver (if using perception)

## Safety Considerations

### Collision Avoidance

- **Arm Separation**: Maintain safe distance between arms
- **Workspace Limits**: Configure non-overlapping safe zones
- **Emergency Stop**: Single e-stop affects both arms
- **Collision Detection**: Monitor for arm-to-arm contact

### Operational Safety

- Test each arm individually before bimanual operation
- Verify CAN ID separation to prevent conflicts
- Monitor total system power consumption
- Ensure adequate cooling for dual motor operation

## Troubleshooting

### Common Issues

1. **CAN ID Conflicts**
   - Verify unique IDs for each motor
   - Check left/right arm ID ranges
   - Monitor CAN traffic: `candump can0`

2. **Controller Conflicts**
   - Ensure proper namespacing
   - Check controller manager status
   - Verify hardware interface allocation

3. **Synchronization Issues**
   - Check system clock synchronization
   - Verify control loop timing
   - Monitor communication latency

### Debugging Commands

```bash
# Check controller status
ros2 control list_controllers

# Monitor joint states
ros2 topic echo /joint_states

# Check hardware interface
ros2 control list_hardware_interfaces
``` 