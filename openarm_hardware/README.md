# OpenArm Hardware Interface Package

This package provides the hardware interface for the OpenArm robotic system, implementing the ros2_control framework for real-time motor control via CAN bus communication.

## Package Contents

### Source Files

- **openarm_hardware.cpp**: Main hardware interface implementation
- **canbus.cpp**: CAN bus communication layer
- **motor.cpp**: Individual motor control and state management
- **motor_control.cpp**: High-level motor control algorithms

### Key Features

- **ros2_control Integration**: Full compatibility with ROS2 control framework
- **CAN Bus Communication**: Real-time communication with motor controllers
- **MIT Control Mode**: Advanced motor control with position, velocity, and torque
- **Multi-Motor Support**: Handles 6-DOF arm configuration plus optional gripper
- **Safety Features**: Torque limiting, position bounds, emergency stop

## Hardware Requirements

### CAN Interface
- **CAN Device**: Typically `can0` or `can1`
- **Protocol**: CAN-FD (CAN with Flexible Data-rate)
- **Baud Rate**: Configured for high-speed motor communication

### Motors
- **Type**: DM series servo motors (DM4310, DM4340, DM6006, DM8006)
- **Communication**: CAN bus with unique device IDs
- **Control Mode**: MIT control (position, velocity, torque)

## Configuration

### Hardware Parameters

The hardware interface accepts the following parameters:

- **can_device**: CAN interface name (e.g., "can0")
- **prefix**: Robot namespace prefix (optional)
- **disable_torque**: Safety flag to disable torque output

### Motor Configuration

Each motor is configured with:
- Motor type (DM4310, DM4340, etc.)
- CAN device ID (unique identifier)
- CAN master ID (control message ID)

## Usage

### Setup CAN Interface

Before using the hardware interface, initialize the CAN bus:

```bash
# See openarm_bringup/utils/init_can.sh
sudo ip link set can0 type can bitrate 1000000
sudo ip link set up can0
```

### Launch with ros2_control

```bash
ros2 launch openarm_bringup openarm.launch.py
```

### Control Interfaces

The hardware interface exports:

**State Interfaces:**
- Position (radians)
- Velocity (rad/s)  
- Effort/Torque (Nm)

**Command Interfaces:**
- Position commands
- Velocity commands
- Effort/Torque feedforward

## Safety Features

- **Position Limits**: Enforced joint position boundaries
- **Velocity Limits**: Maximum joint velocities
- **Torque Limits**: Maximum motor torques
- **Emergency Stop**: Immediate motor disable capability
- **Gradual Startup**: Smooth transition to commanded positions

## Dependencies

- `hardware_interface`
- `rclcpp`
- `rclcpp_lifecycle`
- Linux CAN utilities (`can-utils`)

## Troubleshooting

### Common Issues

1. **CAN Interface Not Found**
   - Check CAN device is up: `ip link show can0`
   - Initialize with proper bitrate

2. **Motor Communication Errors**
   - Verify motor IDs are unique
   - Check CAN bus wiring and termination
   - Monitor CAN traffic: `candump can0`

3. **Control Performance**
   - Tune PID parameters in motor_control.cpp
   - Adjust control loop frequency
   - Check for CAN bus overload 