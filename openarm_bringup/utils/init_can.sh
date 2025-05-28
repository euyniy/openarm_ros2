#!/bin/bash
#
# Copyright 2025 Reazon Holdings, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# This script initializes a CAN interface on a Linux system.
# Usage: ./init_can.sh [OPTIONS] [device] [can_interface]
# 
# Options:
#   -b, --bitrate RATE      Set CAN bitrate (default: 1000000)
#   -d, --dbitrate RATE     Set CAN-FD data bitrate (enables CAN-FD mode)
#   -f, --canfd             Enable CAN-FD mode with default data bitrate 5000000
#   -h, --help              Show this help message
#
# Examples:
#   ./init_can.sh /dev/ttyACM0 can0
#   ./init_can.sh --bitrate 500000 /dev/ttyACM0 can0
#   ./init_can.sh --canfd /dev/ttyACM0 can0
#   ./init_can.sh --bitrate 1000000 --dbitrate 8000000 /dev/ttyACM0 can0

# Default values
BITRATE=1000000
DBITRATE=""
CANFD_MODE=false

# Function to show help
show_help() {
    echo "Usage: $0 [OPTIONS] [device] [can_interface]"
    echo ""
    echo "Initialize a CAN interface on a Linux system."
    echo ""
    echo "Options:"
    echo "  -b, --bitrate RATE      Set CAN bitrate (default: 1000000)"
    echo "  -d, --dbitrate RATE     Set CAN-FD data bitrate (enables CAN-FD mode)"
    echo "  -f, --canfd             Enable CAN-FD mode with default data bitrate 5000000"
    echo "  -h, --help              Show this help message"
    echo ""
    echo "Arguments:"
    echo "  device                  CAN device (default: /dev/ttyACM0)"
    echo "  can_interface           CAN interface name (default: can0)"
    echo ""
    echo "Examples:"
    echo "  $0 /dev/ttyACM0 can0"
    echo "  $0 --bitrate 500000 /dev/ttyACM0 can0"
    echo "  $0 --canfd /dev/ttyACM0 can0"
    echo "  $0 --bitrate 1000000 --dbitrate 8000000 /dev/ttyACM0 can0"
}

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        -b|--bitrate)
            BITRATE="$2"
            shift 2
            ;;
        -d|--dbitrate)
            DBITRATE="$2"
            CANFD_MODE=true
            shift 2
            ;;
        -f|--canfd)
            CANFD_MODE=true
            DBITRATE=${DBITRATE:-5000000}
            shift
            ;;
        -h|--help)
            show_help
            exit 0
            ;;
        -*)
            echo "Unknown option $1"
            show_help
            exit 1
            ;;
        *)
            break
            ;;
    esac
done

# Set device and interface names from remaining arguments
DEVICE=${1:-/dev/ttyACM0}
CAN_INTERFACE=${2:-can0}

echo "Initializing CAN setup:"
echo "  Device: $DEVICE -> $CAN_INTERFACE"
echo "  Bitrate: $BITRATE"
if [ "$CANFD_MODE" = true ]; then
    echo "  CAN-FD mode: enabled"
    echo "  Data bitrate: $DBITRATE"
else
    echo "  CAN-FD mode: disabled"
fi

# Check if device exists
if [ ! -e "$DEVICE" ]; then
    echo "Error: Device $DEVICE does not exist."
    exit 1
fi

echo "Setting up CAN interface..."

# Initialize slcand
sudo slcand -o -c -s8 "$DEVICE"

if [ "$CANFD_MODE" = true ]; then
    # CAN-FD setup
    echo "  Configuring $CAN_INTERFACE for CAN-FD mode..."
    sudo ip link set "$CAN_INTERFACE" down 2>/dev/null || true
    sudo ip link set "$CAN_INTERFACE" type can bitrate $BITRATE dbitrate $DBITRATE fd on
    sudo ip link set "$CAN_INTERFACE" up
else
    # Standard CAN setup
    echo "  Configuring $CAN_INTERFACE for standard CAN mode..."
    sudo ip link set "$CAN_INTERFACE" type can bitrate $BITRATE
    sudo ip link set "$CAN_INTERFACE" up
fi

# Show interface status
sudo ip link show "$CAN_INTERFACE"

echo ""
echo "CAN setup completed successfully!"
echo "$CAN_INTERFACE is ready"

if [ "$CANFD_MODE" = true ]; then
    echo "CAN-FD mode is enabled with data bitrate $DBITRATE"
fi
