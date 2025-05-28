# OpenArm Bimanual Bringup

This package provides launch files and configuration for bringing up the bimanual OpenArm robot system with dual CAN devices.

## Hardware Setup

### CAN Device Initialization

For bimanual operation, you need to initialize two CAN devices:

```bash
# Initialize both CAN devices for bimanual setup (default: 1Mbps)
./utils/init_bimanual_can.sh /dev/ttyACM0 /dev/ttyACM1 can0 can1

# Custom bitrate (500 kbps)
./utils/init_bimanual_can.sh --bitrate 500000 /dev/ttyACM0 /dev/ttyACM1 can0 can1

# Enable CAN-FD mode with default data bitrate (5 Mbps)
./utils/init_bimanual_can.sh --canfd /dev/ttyACM0 /dev/ttyACM1 can0 can1

# Custom CAN-FD bitrates (1 Mbps arbitration, 8 Mbps data)
./utils/init_bimanual_can.sh --bitrate 1000000 --dbitrate 8000000 /dev/ttyACM0 /dev/ttyACM1 can0 can1
```

#### Bitrate Options:
- **Standard CAN**: 125000, 250000, 500000, 1000000 (default)
- **CAN-FD**: Arbitration bitrate + data bitrate (e.g., 1000000 + 5000000)

#### Command Line Options:
- `-b, --bitrate RATE`: Set CAN bitrate (default: 1000000)
- `-d, --dbitrate RATE`: Set CAN-FD data bitrate (enables CAN-FD mode)
- `-f, --canfd`: Enable CAN-FD mode with default data bitrate 5000000
- `-h, --help`: Show help message

Or with custom device names:
```bash
./utils/init_bimanual_can.sh [OPTIONS] [right_device] [left_device] [right_can] [left_can]
```

Default mapping:
- Right arm: `/dev/ttyACM0` → `can0`
- Left arm: `/dev/ttyACM1` → `can1`

### Launch Hardware

```bash
# Launch bimanual hardware with real CAN devices
ros2 launch openarm_bimanual_bringup openarm_bimanual.launch.py

# Launch with mock hardware for testing
ros2 launch openarm_bimanual_bringup openarm_bimanual.launch.py use_mock_hardware:=true

# Launch without RViz
ros2 launch openarm_bimanual_bringup openarm_bimanual.launch.py start_rviz:=false

# Custom CAN devices
ros2 launch openarm_bimanual_bringup openarm_bimanual.launch.py right_can_device:=can2 left_can_device:=can3
```

## Controllers

The following controllers are available:

- `joint_state_broad`: Joint state broadcaster for both arms
- `left_arm_controller`: Joint trajectory controller for left arm
- `right_arm_controller`: Joint trajectory controller for right arm

### Spawning Controllers

Controllers are automatically spawned with the main launch file. To manually spawn:

```bash
# Spawn arm controllers
ros2 run controller_manager spawner left_arm_controller
ros2 run controller_manager spawner right_arm_controller
```

## Configuration

- `config/controllers.yaml`: Controller configuration for both arms
- `utils/init_bimanual_can.sh`: CAN device initialization script

## Dependencies

Make sure you have the following packages installed:
- `can-utils`: For CAN interface management
- `iproute2`: For network interface configuration
- `ros2-control`: ROS2 control framework
- `ros2-controllers`: Standard ROS2 controllers 