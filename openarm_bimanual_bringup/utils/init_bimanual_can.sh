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
# This script initializes multiple CAN interfaces for bimanual OpenArm setup.
# Usage: ./init_bimanual_can.sh [OPTIONS] [right_device] [left_device] [right_can] [left_can]
# 
# Options:
#   -b, --bitrate RATE      Set CAN bitrate (default: 1000000)
#   -d, --dbitrate RATE     Set CAN-FD data bitrate (enables CAN-FD mode)
#   -f, --canfd             Enable CAN-FD mode with default data bitrate 5000000
#   -h, --help              Show this help message
#
# Examples:
#   ./init_bimanual_can.sh /dev/ttyACM0 /dev/ttyACM1 can0 can1
#   ./init_bimanual_can.sh --bitrate 500000 /dev/ttyACM0 /dev/ttyACM1 can0 can1
#   ./init_bimanual_can.sh --canfd /dev/ttyACM0 /dev/ttyACM1 can0 can1
#   ./init_bimanual_can.sh --bitrate 1000000 --dbitrate 8000000 /dev/ttyACM0 /dev/ttyACM1 can0 can1

# Default values
BITRATE=1000000
DBITRATE=""
CANFD_MODE=false

# Function to show help
show_help() {
    echo "Usage: $0 [OPTIONS] [right_device] [left_device] [right_can] [left_can]"
    echo ""
    echo "Initialize multiple CAN interfaces for bimanual OpenArm setup."
    echo ""
    echo "Options:"
    echo "  -b, --bitrate RATE      Set CAN bitrate (default: 1000000)"
    echo "  -d, --dbitrate RATE     Set CAN-FD data bitrate (enables CAN-FD mode)"
    echo "  -f, --canfd             Enable CAN-FD mode with default data bitrate 5000000"
    echo "  -h, --help              Show this help message"
    echo ""
    echo "Arguments:"
    echo "  right_device            Right arm device (default: /dev/ttyACM0)"
    echo "  left_device             Left arm device (default: /dev/ttyACM1)"
    echo "  right_can               Right arm CAN interface (default: can0)"
    echo "  left_can                Left arm CAN interface (default: can1)"
    echo ""
    echo "Examples:"
    echo "  $0 /dev/ttyACM0 /dev/ttyACM1 can0 can1"
    echo "  $0 --bitrate 500000 /dev/ttyACM0 /dev/ttyACM1 can0 can1"
    echo "  $0 --canfd /dev/ttyACM0 /dev/ttyACM1 can0 can1"
    echo "  $0 --bitrate 1000000 --dbitrate 8000000 /dev/ttyACM0 /dev/ttyACM1 can0 can1"
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
RIGHT_DEVICE=${1:-/dev/ttyACM0}
LEFT_DEVICE=${2:-/dev/ttyACM1}
RIGHT_CAN=${3:-can0}
LEFT_CAN=${4:-can1}

echo "Initializing bimanual CAN setup:"
echo "  Right arm: $RIGHT_DEVICE -> $RIGHT_CAN"
echo "  Left arm:  $LEFT_DEVICE -> $LEFT_CAN"
echo "  Bitrate: $BITRATE"
if [ "$CANFD_MODE" = true ]; then
    echo "  CAN-FD mode: enabled"
    echo "  Data bitrate: $DBITRATE"
else
    echo "  CAN-FD mode: disabled"
fi

# Check if devices exist
if [ ! -e "$RIGHT_DEVICE" ]; then
    echo "Error: Right arm device $RIGHT_DEVICE does not exist."
    exit 1
fi

if [ ! -e "$LEFT_DEVICE" ]; then
    echo "Error: Left arm device $LEFT_DEVICE does not exist."
    exit 1
fi

# Function to setup CAN interface
setup_can_interface() {
    local device=$1
    local can_interface=$2
    local arm_name=$3
    
    echo "Setting up $arm_name CAN interface..."
    
    # Initialize slcand
    sudo slcand -o -c -s8 "$device"
    
    if [ "$CANFD_MODE" = true ]; then
        # CAN-FD setup
        echo "  Configuring $can_interface for CAN-FD mode..."
        sudo ip link set "$can_interface" down 2>/dev/null || true
        sudo ip link set "$can_interface" type can bitrate $BITRATE dbitrate $DBITRATE fd on
        sudo ip link set "$can_interface" up
    else
        # Standard CAN setup
        echo "  Configuring $can_interface for standard CAN mode..."
        sudo ip link set "$can_interface" type can bitrate $BITRATE
        sudo ip link set "$can_interface" up
    fi
    
    # Show interface status
    sudo ip link show "$can_interface"
}

# Initialize right arm CAN
setup_can_interface "$RIGHT_DEVICE" "$RIGHT_CAN" "right arm"

# Initialize left arm CAN
setup_can_interface "$LEFT_DEVICE" "$LEFT_CAN" "left arm"

echo ""
echo "Bimanual CAN setup completed successfully!"
echo "Right arm: $RIGHT_CAN is ready"
echo "Left arm:  $LEFT_CAN is ready"

if [ "$CANFD_MODE" = true ]; then
    echo "CAN-FD mode is enabled with data bitrate $DBITRATE"
fi 