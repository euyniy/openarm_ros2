# OpenArm Bimanual Description Package

This package contains the URDF models and visualization configurations for the dual-arm OpenArm robotic system, including torso and RealSense head camera.

## Package Contents

### Directories

- **urdf/**: URDF and xacro files for the bimanual robot system
- **meshes/**: 3D mesh files for visual and collision representations
- **launch/**: Launch files for visualization and simulation
- **rviz/**: RViz configuration files for dual-arm visualization
- **worlds/**: Gazebo world files for bimanual simulation

### Key Features

- **Dual Arm Configuration**: Two OpenArm manipulators mounted on a torso
- **Torso Base**: Central mounting platform for both arms
- **RealSense Camera**: Head-mounted depth camera for perception
- **Coordinated Workspace**: Optimized arm placement for bimanual manipulation

## Usage

### Visualize Bimanual System

```bash
ros2 launch openarm_bimanual_description display.launch.py
```

### Simulate in Gazebo

```bash
ros2 launch openarm_bimanual_description gazebo.launch.py
```

## Robot System Specifications

- **Total DOF**: 12 (6 per arm)
- **Workspace**: Overlapping manipulation zones for bimanual tasks
- **Sensors**: RealSense D435 depth camera
- **Base**: Fixed torso platform
- **Arm Separation**: Optimized for human-like bimanual manipulation

## Coordinate Frames

- **base_link**: Central torso reference frame
- **left_arm_base_link**: Left arm mounting point
- **right_arm_base_link**: Right arm mounting point
- **camera_link**: RealSense camera frame

## Dependencies

- `openarm_description`: Single arm models
- `robot_state_publisher`
- `joint_state_publisher`
- `rviz2`
- `gazebo_ros_pkgs`
- `realsense2_description`

## Applications

This bimanual configuration is designed for:
- Dual-arm manipulation tasks
- Object handover between arms
- Coordinated assembly operations
- Human-robot interaction scenarios
- Research in bimanual robotics 