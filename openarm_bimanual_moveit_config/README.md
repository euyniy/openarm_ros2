# OpenArm Bimanual MoveIt Configuration Package

This package contains the MoveIt2 motion planning configuration for the dual-arm OpenArm robotic system, enabling coordinated bimanual manipulation with advanced path planning and collision detection.

## Package Contents

### Directories

- **config/**: MoveIt configuration files for bimanual system
  - **kinematics.yaml**: Kinematic solvers for both arms
  - **joint_limits.yaml**: Joint limits for left and right arms
  - **openarm_bimanual.srdf**: Semantic robot description
  - **sensors_3d.yaml**: RealSense camera configuration
  - **moveit_controllers.yaml**: Dual-arm controller setup

- **launch/**: Launch files for bimanual MoveIt functionality
  - **demo.launch.py**: Interactive bimanual demo in RViz
  - **move_group.launch.py**: Bimanual move_group server
  - **planning_context.launch.py**: Dual-arm planning context

## Key Features

### Bimanual Motion Planning

- **Coordinated Planning**: Simultaneous motion planning for both arms
- **Independent Control**: Individual arm planning and execution
- **Collision Avoidance**: Inter-arm collision detection and avoidance
- **Workspace Coordination**: Optimized dual-arm workspace utilization

### Planning Groups

- **left_arm**: Left 6-DOF manipulator group
- **right_arm**: Right 6-DOF manipulator group
- **both_arms**: Combined bimanual planning group
- **left_gripper**: Left end-effector (if equipped)
- **right_gripper**: Right end-effector (if equipped)

### Advanced Capabilities

- **OctoMap Integration**: Real-time 3D occupancy mapping
- **RealSense Camera**: Head-mounted depth perception
- **Dynamic Obstacles**: Real-time environment updates
- **Multi-Goal Planning**: Complex bimanual task execution

## Usage

### Prerequisites

Ensure the ROS2 packages and dependencies are installed by following the instructions in `openarm_ros2/README.md`.

### Hardware Setup

1. **Initialize CAN Bus**: Run `init_can.sh` from `openarm_bringup/utils`
   
   By default, can0 is the right arm and can1 is the left arm, but this can be adjusted in the ros2_control definition in `openarm_description/urdf/openarm.ros2_control.xacro`.

   ```bash
   cd openarm_bringup/utils
   ./init_can.sh /dev/ttyACM0 can0
   ./init_can.sh /dev/ttyACM1 can1
   ```

2. **Start RealSense Camera** (Optional): Enable octomap occupancy grid for planning around obstacles
   
   ```bash
   ros2 launch openarm_bimanual_bringup depth_camera.launch.py
   ```

### Launch Demo

Interactive bimanual planning demo:

```bash
ros2 launch openarm_bimanual_moveit_config demo.launch.py
```

### With Real Hardware

```bash
# Terminal 1: Start bimanual hardware
ros2 launch openarm_bimanual_bringup bimanual.launch.py

# Terminal 2: Start MoveIt
ros2 launch openarm_bimanual_moveit_config move_group.launch.py

# Terminal 3: Start RViz (optional)
ros2 launch openarm_bimanual_moveit_config moveit_rviz.launch.py
```

## Programming Interface

### Python API - Coordinated Motion

```python
import rclpy
from moveit_py import MoveItPy

# Initialize bimanual MoveIt
moveit = MoveItPy(node_name="bimanual_moveit")
left_arm = moveit.get_planning_component("left_arm")
right_arm = moveit.get_planning_component("right_arm")
both_arms = moveit.get_planning_component("both_arms")

# Coordinated bimanual motion
both_arms.set_start_state_to_current_state()
both_arms.set_goal_state(configuration_name="bimanual_ready")
plan_result = both_arms.plan()

if plan_result:
    both_arms.execute(plan_result.trajectory)
```

### Python API - Independent Arms

```python
# Independent arm control
left_arm.set_start_state_to_current_state()
right_arm.set_start_state_to_current_state()

# Set different goals for each arm
left_arm.set_goal_state(configuration_name="left_home")
right_arm.set_goal_state(configuration_name="right_ready")

# Plan and execute independently
left_plan = left_arm.plan()
right_plan = right_arm.plan()

if left_plan and right_plan:
    left_arm.execute(left_plan.trajectory)
    right_arm.execute(right_plan.trajectory)
```

## Configuration Details

### Bimanual Planning Groups

- **both_arms**: Combined 12-DOF planning group for coordinated motion
- **left_arm**: Independent 6-DOF left arm planning
- **right_arm**: Independent 6-DOF right arm planning

### Collision Detection

- **Self-Collision**: Individual arm link collision checking
- **Inter-Arm Collision**: Left-right arm collision avoidance
- **Environment Collision**: Obstacle avoidance using OctoMap
- **Allowed Collisions**: Configured safe contact pairs

### Named Poses

Pre-defined bimanual configurations:
- **bimanual_home**: Both arms in safe home position
- **bimanual_ready**: Ready for manipulation tasks
- **handover_pose**: Optimized for object transfer between arms
- **tucked**: Compact storage position for both arms

## Perception Integration

### RealSense Camera Setup

The head-mounted RealSense D435 provides:
- **Depth Perception**: 3D point cloud data
- **OctoMap Updates**: Real-time occupancy grid mapping
- **Obstacle Detection**: Dynamic environment awareness

### OctoMap Configuration

```yaml
octomap:
  resolution: 0.025
  frame_id: "base_link"
  max_range: 4.0
  sensor_model:
    hit: 0.7
    miss: 0.4
    min: 0.12
    max: 0.97
```

## Bimanual Applications

This configuration enables:

- **Object Handover**: Passing objects between arms
- **Coordinated Assembly**: Both arms working on same task
- **Dual Manipulation**: Independent simultaneous tasks
- **Human-Robot Interaction**: Natural bimanual behaviors
- **Complex Assembly**: Multi-handed manufacturing tasks

## Dependencies

- `openarm_bimanual_description`: Dual-arm robot model
- `moveit_core`: Core MoveIt functionality
- `moveit_ros_planning`: Planning infrastructure
- `moveit_planners_ompl`: OMPL planning algorithms
- `moveit_servo`: Real-time servoing
- `octomap_server`: 3D occupancy mapping
- `realsense2_camera`: Camera driver

## Troubleshooting

### Common Issues

1. **Bimanual Planning Failures**
   - Check both arms are in valid start states
   - Verify inter-arm collision settings
   - Increase planning time for complex motions

2. **OctoMap Issues**
   - Verify RealSense camera is publishing
   - Check point cloud topic names
   - Adjust octomap resolution and range

3. **Controller Conflicts**
   - Ensure proper arm namespacing
   - Check controller manager status
   - Verify hardware interface allocation

### Debugging Commands

```bash
# Check bimanual planning groups
ros2 service call /get_planning_scene moveit_msgs/srv/GetPlanningScene

# Monitor joint states for both arms
ros2 topic echo /joint_states

# Check octomap updates
ros2 topic echo /octomap_binary

# Verify camera data
ros2 topic echo /camera/depth/color/points
``` 