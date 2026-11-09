"""Mapless Navigation 2 (Nav2) launch file for Robot 1.

Starts the Nav2 stack in rolling-window odometry mode (`global_frame: odom`)
without any static arena map, enabling direct click-to-go ("2D Goal Pose") in
RViz2 with real-time Duplo brick avoidance via the Hokuyo URG-04LX `/scan`.
"""

import os
from typing import List

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Package & File Constants
BRINGUP_PACKAGE_NAME: str = "robot1_bringup"
NAV2_PARAMS_REL_PATH: str = os.path.join("config", "nav2_params.yaml")

# Launch Argument Constants
USE_SIM_TIME_ARG: str = "use_sim_time"
DEFAULT_USE_SIM_TIME: str = "true"
PARAMS_FILE_ARG: str = "params_file"

# Static Map -> Odom Transform Constants
MAP_FRAME_ID: str = "map"
ODOM_FRAME_ID: str = "odom"

# Lifecycle-Managed Nav2 Server Node Names
LIFECYCLE_MANAGED_NODES: List[str] = [
    "controller_server",
    "planner_server",
    "behavior_server",
    "bt_navigator",
]


def generate_launch_description() -> LaunchDescription:
    """Create the LaunchDescription for mapless Nav2 autonomous navigation.

    Returns:
        Configured LaunchDescription starting a static `map -> odom` identity
        transform, `controller_server`, `planner_server`, `behavior_server`,
        `bt_navigator`, and `nav2_lifecycle_manager`.
    """
    bringup_share_dir: str = get_package_share_directory(BRINGUP_PACKAGE_NAME)
    default_params_path: str = os.path.join(bringup_share_dir, NAV2_PARAMS_REL_PATH)

    declare_use_sim_time = DeclareLaunchArgument(
        name=USE_SIM_TIME_ARG,
        default_value=DEFAULT_USE_SIM_TIME,
        description="Synchronize Nav2 servers with Gazebo /clock if true.",
    )
    declare_params_file = DeclareLaunchArgument(
        name=PARAMS_FILE_ARG,
        default_value=default_params_path,
        description="Absolute path to the Nav2 parameters YAML file.",
    )

    use_sim_time = LaunchConfiguration(USE_SIM_TIME_ARG)
    params_file = LaunchConfiguration(PARAMS_FILE_ARG)

    # Static identity transform map -> odom so both frames remain valid in RViz2
    static_map_to_odom_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_map_to_odom_publisher",
        output="screen",
        arguments=[
            "--x",
            "0.0",
            "--y",
            "0.0",
            "--z",
            "0.0",
            "--roll",
            "0.0",
            "--pitch",
            "0.0",
            "--yaw",
            "0.0",
            "--frame-id",
            MAP_FRAME_ID,
            "--child-frame-id",
            ODOM_FRAME_ID,
        ],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    controller_server_node = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
    )

    planner_server_node = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
    )

    behavior_server_node = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
    )

    bt_navigator_node = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
    )

    lifecycle_manager_node = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "autostart": True,
                "node_names": LIFECYCLE_MANAGED_NODES,
            }
        ],
    )

    return LaunchDescription(
        [
            declare_use_sim_time,
            declare_params_file,
            static_map_to_odom_node,
            controller_server_node,
            planner_server_node,
            behavior_server_node,
            bt_navigator_node,
            lifecycle_manager_node,
        ]
    )
