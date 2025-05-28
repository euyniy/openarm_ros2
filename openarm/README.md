# OpenArm Metapackage

This is the main metapackage for the OpenArm robotic arm system. It provides a convenient way to install and manage all OpenArm-related ROS2 packages.

## Overview

The OpenArm metapackage includes the following sub-packages:

- **openarm_description**: URDF models and visualization for single arm configuration
- **openarm_bimanual_description**: URDF models for dual arm configuration with torso and camera
- **openarm_hardware**: Hardware interface for ros2_control integration
- **openarm_moveit_config**: MoveIt2 motion planning configuration for single arm
- **openarm_bimanual_moveit_config**: MoveIt2 motion planning configuration for dual arm
- **openarm_bringup**: Launch files and utilities for single arm setup
- **openarm_bimanual_bringup**: Launch files and utilities for dual arm setup

## Installation

This metapackage automatically installs all required dependencies when built with colcon:

```bash
cd ~/ros2_ws
colcon build --packages-select openarm
```

## Usage

After installation, you can use any of the individual packages. For quick testing:

```bash
# Single arm demo
ros2 launch openarm_moveit_config demo.launch.py

# Dual arm demo  
ros2 launch openarm_bimanual_moveit_config demo.launch.py
```

## Version

Current version: 0.3.0

## License

Apache License 2.0 