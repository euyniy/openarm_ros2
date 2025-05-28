# OpenArm Bringup

This package provides launch files and configuration for bringing up a single OpenArm robot.

## CAN Device Initialization

### Basic Usage

```bash
# Initialize CAN device with default settings (1 Mbps)
./utils/init_can.sh /dev/ttyACM0 can0

# Custom bitrate (500 kbps)
./utils/init_can.sh --bitrate 500000 /dev/ttyACM0 can0

# Enable CAN-FD mode with default data bitrate (5 Mbps)
./utils/init_can.sh --canfd /dev/ttyACM0 can0

# Custom CAN-FD bitrates (1 Mbps arbitration, 8 Mbps data)
./utils/init_can.sh --bitrate 1000000 --dbitrate 8000000 /dev/ttyACM0 can0
```

### Command Line Options

- `-b, --bitrate RATE`: Set CAN bitrate (default: 1000000)
- `-d, --dbitrate RATE`: Set CAN-FD data bitrate (enables CAN-FD mode)
- `-f, --canfd`: Enable CAN-FD mode with default data bitrate 5000000
- `-h, --help`: Show help message

### Supported Bitrates

- **Standard CAN**: 125000, 250000, 500000, 1000000 (default)
- **CAN-FD**: Arbitration bitrate + data bitrate (e.g., 1000000 + 5000000)

### Examples

```bash
# Standard CAN at different bitrates
./utils/init_can.sh --bitrate 125000 /dev/ttyACM0 can0
./utils/init_can.sh --bitrate 250000 /dev/ttyACM0 can0
./utils/init_can.sh --bitrate 500000 /dev/ttyACM0 can0
./utils/init_can.sh --bitrate 1000000 /dev/ttyACM0 can0

# CAN-FD examples
./utils/init_can.sh --canfd /dev/ttyACM0 can0                                    # 1M + 5M
./utils/init_can.sh --bitrate 1000000 --dbitrate 8000000 /dev/ttyACM0 can0      # 1M + 8M
./utils/init_can.sh --bitrate 500000 --dbitrate 4000000 /dev/ttyACM0 can0       # 500k + 4M
```
