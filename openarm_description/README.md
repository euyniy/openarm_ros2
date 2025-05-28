# OpenArm Description Package

This package contains the URDF (Unified Robot Description Format) models, meshes, and visualization configurations for the single OpenArm robotic arm.

## Package Contents

### Directories

- **urdf/**: URDF and xacro files defining the robot model
- **meshes/**: 3D mesh files for visual and collision representations
- **launch/**: Launch files for visualization and simulation
- **config/**: Configuration files for robot parameters
- **rviz/**: RViz configuration files for visualization
- **worlds/**: Gazebo world files for simulation
- **resource/**: Additional resource files

### Key Files

- **urdf/openarm.urdf.xacro**: Main robot description file
- **launch/display.launch.py**: Launch RViz with robot model
- **launch/gazebo.launch.py**: Launch Gazebo simulation
- **launch/description.launch.py**: Robot state publisher launch file

## Usage

### Visualize in RViz

```bash
ros2 launch openarm_description display.launch.py
```

### Simulate in Gazebo

```bash
ros2 launch openarm_description gazebo.launch.py
```

### Use in other packages

```bash
ros2 launch openarm_description description.launch.py
```

## Robot Model Features

- **Degrees of Freedom**: 6-DOF arm configuration
- **Visual Meshes**: High-quality 3D models for visualization
- **Collision Meshes**: Simplified geometry for collision detection
- **Joint Limits**: Properly configured joint position, velocity, and effort limits
- **Inertial Properties**: Mass and inertia properties for realistic simulation

## Dependencies

- `robot_state_publisher`
- `joint_state_publisher`
- `joint_state_publisher_gui`
- `rviz2`
- `gazebo_ros_pkgs`
- `xacro`

## Configuration

The robot model can be customized through parameters in the launch files:
- Robot prefix for multi-robot setups
- Simulation vs real hardware configurations
- Additional sensors and end-effectors 