"""Complete simulation bringup launch file for Robot 1 in the Robogames arena.

This launch file orchestrates:
1. Gazebo resource path registration for `robot1_description` and `myrobot_description` meshes.
2. Inclusion of `robogames_arena_gazebo` (world + target box + 6 Duplo bricks).
3. Inclusion of `robot1_description` (`robot_state_publisher`).
4. Spawning `robot1` inside the bottom-left robot starting zone.
5. Starting `ros_gz_bridge` (`/clock`, `/cmd_vel`, `/odom`, `/tf`, `/joint_states`, `/scan_raw`).
6. Starting `laser_self_filter` (`/scan_raw` -> wheel-filtered `/scan`).
7. Starting mapless Nav2 (`navigation.launch.py`, enabled by default via `use_nav2:=true`).
8. Starting `rviz2` pre-configured with `2D Goal Pose` (`/goal_pose`), `/scan`, and `/plan`.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Package Names
BRINGUP_PACKAGE_NAME: str = "robot1_bringup"
DESCRIPTION_PACKAGE_NAME: str = "robot1_description"
ARM_DESCRIPTION_PACKAGE_NAME: str = "myrobot_description"
ARENA_PACKAGE_NAME: str = "robogames_arena_gazebo"

# Relative Asset Paths
ARENA_LAUNCH_REL_PATH: str = os.path.join("launch", "start_competition.launch.py")
DESCRIPTION_LAUNCH_REL_PATH: str = os.path.join("launch", "description.launch.py")
NAVIGATION_LAUNCH_REL_PATH: str = os.path.join("launch", "navigation.launch.py")
BRIDGE_CONFIG_REL_PATH: str = os.path.join("config", "ros_gz_bridge.yaml")
RVIZ_CONFIG_REL_PATH: str = os.path.join("rviz", "robot1_lidar_test.rviz")

# Robot Spawning Constants (Centered inside Bottom-Left Start Zone [-0.70..-0.35, -0.50..-0.15])
ROBOT_ENTITY_NAME: str = "robot1"
ROBOT_DESCRIPTION_TOPIC: str = "robot_description"
ROBOT_SPAWN_X: float = -0.525
ROBOT_SPAWN_Y: float = -0.325
# Playing field surface sits at Z = 0.010 m; spawning at 0.015 m settles cleanly by 5 mm
ROBOT_SPAWN_Z: float = 0.015
ROBOT_SPAWN_YAW_RAD: float = 0.62

# Launch Argument Constants
USE_RVIZ_ARG: str = "use_rviz"
DEFAULT_USE_RVIZ: str = "true"
USE_NAV2_ARG: str = "use_nav2"
DEFAULT_USE_NAV2: str = "true"
USE_SIM_TIME_ARG: str = "use_sim_time"
DEFAULT_USE_SIM_TIME: str = "true"


def generate_launch_description() -> LaunchDescription:
    """Assemble the full Gazebo Harmonic + ROS 2 Jazzy + Nav2 simulation stack.

    Returns:
        Configured LaunchDescription launching the arena, robot, ROS-Gazebo
        topic bridge, LiDAR wheel self-filter, mapless Nav2, and RViz2.
    """
    bringup_share_dir: str = get_package_share_directory(BRINGUP_PACKAGE_NAME)
    description_share_dir: str = get_package_share_directory(DESCRIPTION_PACKAGE_NAME)
    arm_description_share_dir: str = get_package_share_directory(
        ARM_DESCRIPTION_PACKAGE_NAME
    )
    arena_share_dir: str = get_package_share_directory(ARENA_PACKAGE_NAME)

    declare_use_rviz = DeclareLaunchArgument(
        name=USE_RVIZ_ARG,
        default_value=DEFAULT_USE_RVIZ,
        description="Launch RViz2 with 2D Goal Pose and Hokuyo LaserScan view if true.",
    )
    declare_use_nav2 = DeclareLaunchArgument(
        name=USE_NAV2_ARG,
        default_value=DEFAULT_USE_NAV2,
        description="Launch mapless Nav2 autonomous navigation stack if true.",
    )
    declare_use_sim_time = DeclareLaunchArgument(
        name=USE_SIM_TIME_ARG,
        default_value=DEFAULT_USE_SIM_TIME,
        description="Synchronize all ROS 2 nodes with Gazebo /clock.",
    )

    use_rviz_config = LaunchConfiguration(USE_RVIZ_ARG)
    use_nav2_config = LaunchConfiguration(USE_NAV2_ARG)
    use_sim_time_config = LaunchConfiguration(USE_SIM_TIME_ARG)

    # 1. Expose robot1_description and myrobot_description parent share directories
    #    so Gazebo resolves package:// meshes for both the Hokuyo LiDAR and the 5-DOF arm
    description_parent_share_dir: str = os.path.dirname(description_share_dir)
    arm_parent_share_dir: str = os.path.dirname(arm_description_share_dir)
    gz_resource_env_robot = AppendEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=description_parent_share_dir,
    )
    gz_resource_env_arm = AppendEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=arm_parent_share_dir,
    )

    # 2. Launch Robogames arena world + 6 randomized Duplo bricks
    arena_launch_path: str = os.path.join(arena_share_dir, ARENA_LAUNCH_REL_PATH)
    arena_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(arena_launch_path)
    )

    # 3. Publish Robot 1 URDF/Xacro via robot_state_publisher
    description_launch_path: str = os.path.join(
        description_share_dir, DESCRIPTION_LAUNCH_REL_PATH
    )
    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(description_launch_path),
        launch_arguments={USE_SIM_TIME_ARG: use_sim_time_config}.items(),
    )

    # 4. Spawn Robot 1 inside the bottom-left start zone facing the arena center
    spawn_robot_node = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_robot1",
        output="screen",
        arguments=[
            "-name",
            ROBOT_ENTITY_NAME,
            "-topic",
            ROBOT_DESCRIPTION_TOPIC,
            "-x",
            f"{ROBOT_SPAWN_X:.4f}",
            "-y",
            f"{ROBOT_SPAWN_Y:.4f}",
            "-z",
            f"{ROBOT_SPAWN_Z:.4f}",
            "-Y",
            f"{ROBOT_SPAWN_YAW_RAD:.4f}",
        ],
    )

    # 5. Bidirectional ROS 2 <-> Gazebo Harmonic bridge
    bridge_config_path: str = os.path.join(bringup_share_dir, BRIDGE_CONFIG_REL_PATH)
    ros_gz_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_bridge",
        output="screen",
        parameters=[
            {
                "config_file": bridge_config_path,
                "use_sim_time": use_sim_time_config,
            }
        ],
    )

    # 6. Filter out front-wheel self-detections from /scan_raw into /scan
    laser_self_filter_node = Node(
        package=BRINGUP_PACKAGE_NAME,
        executable="laser_self_filter",
        name="laser_self_filter",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time_config}],
    )

    # 7. Mapless Nav2 stack (click-to-go in RViz2 without static arena boundary map)
    navigation_launch_path: str = os.path.join(
        bringup_share_dir, NAVIGATION_LAUNCH_REL_PATH
    )
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(navigation_launch_path),
        launch_arguments={USE_SIM_TIME_ARG: use_sim_time_config}.items(),
        condition=IfCondition(use_nav2_config),
    )

    # 8. RViz2 visualization showing RobotModel, TF, Hokuyo /scan, and Nav2 /plan
    rviz_config_path: str = os.path.join(description_share_dir, RVIZ_CONFIG_REL_PATH)
    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_path],
        parameters=[{"use_sim_time": use_sim_time_config}],
        condition=IfCondition(use_rviz_config),
    )

    return LaunchDescription(
        [
            declare_use_rviz,
            declare_use_nav2,
            declare_use_sim_time,
            gz_resource_env_robot,
            gz_resource_env_arm,
            arena_simulation,
            robot_description_launch,
            spawn_robot_node,
            ros_gz_bridge_node,
            laser_self_filter_node,
            navigation_launch,
            rviz2_node,
        ]
    )
