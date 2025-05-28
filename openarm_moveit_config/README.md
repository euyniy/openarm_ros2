# OpenArm MoveIt Configuration Package

This package contains the MoveIt2 motion planning configuration for the single OpenArm robotic arm, providing advanced path planning, collision detection, and manipulation capabilities.

## Package Contents

### Directories

- **config/**: MoveIt configuration files
  - **kinematics.yaml**: Kinematic solver configuration
  - **joint_limits.yaml**: Joint velocity and acceleration limits
  - **openarm.srdf**: Semantic Robot Description Format
  - **sensors_3d.yaml**: 3D sensor configuration
  - **moveit_controllers.yaml**: Controller configuration

- **launch/**: Launch files for MoveIt functionality
  - **demo.launch.py**: Interactive MoveIt demo in RViz
  - **move_group.launch.py**: Core MoveIt move_group server
  - **planning_context.launch.py**: Planning scene configuration

## Key Features

### Motion Planning

- **OMPL Integration**: Open Motion Planning Library algorithms
- **Multiple Planners**: RRT, RRTConnect, EST, PRM, and more
- **Collision Detection**: Real-time collision checking
- **Path Optimization**: Smooth, efficient trajectories

### Planning Groups

- **arm**: Main 6-DOF manipulator group
- **gripper**: End-effector group (if equipped)

### Kinematic Solvers

- **Forward Kinematics**: Joint angles to end-effector pose
- **Inverse Kinematics**: End-effector pose to joint angles
- **Solver**: KDL (Kinematics and Dynamics Library)

## Usage

### Interactive Demo

Launch MoveIt with interactive planning in RViz:

```bash
ros2 launch openarm_moveit_config demo.launch.py
```

### Move Group Server

Start the core MoveIt planning server:

```bash
ros2 launch openarm_moveit_config move_group.launch.py
```

### With Real Hardware

Combine with hardware bringup:

```bash
# Terminal 1: Start hardware
ros2 launch openarm_bringup openarm.launch.py

# Terminal 2: Start MoveIt
ros2 launch openarm_moveit_config move_group.launch.py
```

## Programming Interface

### Python API

```python
import rclpy
from moveit_py import MoveItPy

# Initialize MoveIt
moveit = MoveItPy(node_name="moveit_py")
arm = moveit.get_planning_component("arm")

# Plan to named target
arm.set_start_state_to_current_state()
arm.set_goal_state(configuration_name="home")
plan_result = arm.plan()

# Execute motion
if plan_result:
    arm.execute(plan_result.trajectory)
```

### C++ API

```cpp
#include <moveit/move_group_interface/move_group_interface.h>

// Initialize move group
moveit::planning_interface::MoveGroupInterface move_group("arm");

// Set target pose
geometry_msgs::msg::Pose target_pose;
target_pose.position.x = 0.3;
target_pose.position.y = 0.0;
target_pose.position.z = 0.5;
move_group.setPoseTarget(target_pose);

// Plan and execute
move_group.move();
```

## Configuration Details

### Planning Algorithms

Available OMPL planners:
- **RRTConnect**: Fast, bidirectional RRT
- **RRT**: Rapidly-exploring Random Tree
- **EST**: Expansive Space Trees
- **PRM**: Probabilistic Roadmap Method
- **BKPIECE**: Bi-directional KPIECE
- **LBKPIECE**: Lazy Bi-directional KPIECE

### Joint Limits

Configured limits for safe operation:
- **Position Limits**: Joint angle boundaries
- **Velocity Limits**: Maximum joint speeds
- **Acceleration Limits**: Maximum joint accelerations
- **Jerk Limits**: Smooth motion profiles

### Collision Detection

- **Self-Collision**: Arm links collision checking
- **Environment Collision**: Obstacle avoidance
- **Allowed Collisions**: Permitted contact pairs
- **Safety Margins**: Collision buffer distances

## Named Poses

Pre-defined robot configurations:
- **home**: Safe starting position
- **ready**: Manipulation ready pose
- **tucked**: Compact storage position

## Integration with Perception

### 3D Sensors

Configure depth cameras for obstacle detection:

```yaml
sensors:
  - sensor_plugin: occupancy_map_monitor/PointCloudOctomapUpdater
    point_cloud_topic: /camera/depth/color/points
    max_range: 5.0
    point_subsample: 1
```

### Octomap

Real-time 3D occupancy mapping:
- **Resolution**: Configurable voxel size
- **Update Rate**: Real-time obstacle updates
- **Decay**: Automatic obstacle removal

## Dependencies

- `moveit_core`
- `moveit_ros_planning`
- `moveit_ros_planning_interface`
- `moveit_planners_ompl`
- `moveit_servo`
- `openarm_description`

## Troubleshooting

### Common Issues

1. **Planning Failures**
   - Check joint limits in SRDF
   - Verify start/goal states are valid
   - Increase planning time limit

2. **Collision Detection Issues**
   - Review collision meshes
   - Check allowed collision matrix
   - Adjust safety margins

3. **IK Solver Problems**
   - Verify kinematic chain
   - Check joint limits
   - Try different IK solvers 