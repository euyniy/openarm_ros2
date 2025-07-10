# from moveit_configs_utils import MoveItConfigsBuilder
# from moveit_configs_utils.launches import generate_demo_launch

# def generate_launch_description():
#     moveit_config = (
#         MoveItConfigsBuilder("openarm",
#                              package_name="openarm_description_moveit_config")
#         .robot_description(file_path="config/openarm.urdf.xacro")        # explicit
#         .robot_description_semantic(file_path="config/openarm.srdf")     # explicit
#         .to_moveit_configs()
#     )
#     return (generate_demo_launch(moveit_config))


#!/usr/bin/env python3
"""
demo.launch.py  –  MoveIt-RViz demo **plus** ros2_control_node
"""

import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description() -> LaunchDescription:
    # ------------------------------------------------------------------
    # 1. Build the MoveIt configuration (same as before)
    # ------------------------------------------------------------------
    moveit_config = (
        MoveItConfigsBuilder("openarm",
                             package_name="openarm_description_moveit_config")
        #  ▽ give the explicit paths only if you need to override defaults
        # .robot_description(file_path="config/openarm.urdf.xacro")
        # .robot_description_semantic(file_path="config/openarm.srdf")
        .to_moveit_configs()
    )

    moveit_demo_launch = generate_demo_launch(moveit_config)

    # ------------------------------------------------------------------
    # 2. Start ros2_control_node with your controller YAML
    # ------------------------------------------------------------------
    pkg_share = get_package_share_directory("openarm_description_moveit_config")
    ros2_controllers_path = os.path.join(pkg_share, "config",
                                         "ros2_controllers.yaml")

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[  # pass both controller-manager params and the URDF
            ros2_controllers_path,
            {"robot_description": moveit_config.robot_description},
        ],
        # the spawner helpers look for /controller_manager/robot_description,
        # so expose the same parameter under that namespace as well
        remappings=[
            ("/controller_manager/robot_description", "/robot_description")
        ],
        output="both",
        namespace="",
    )

    # ------------------------------------------------------------------
    # 3. Return everything as one LaunchDescription
    # ------------------------------------------------------------------
    ld = LaunchDescription()
    ld.add_action(moveit_demo_launch)   # MoveIt, RViz, spawners
    # ld.add_action(ros2_control_node)    # controller manager itself
    return ld
