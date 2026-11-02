"""Launch file for publishing the Robot 1 URDF/Xacro description and TF tree.

This launch file compiles `robot1.urdf.xacro` at runtime and starts
`robot_state_publisher` so both Gazebo Sim and RViz2 share a single canonical
robot model.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# Package and File Constants
DESCRIPTION_PACKAGE_NAME: str = "robot1_description"
URDF_SUBDIRECTORY: str = "urdf"
ROOT_XACRO_FILENAME: str = "robot1.urdf.xacro"

# Launch Argument Constants
USE_SIM_TIME_ARG: str = "use_sim_time"
DEFAULT_USE_SIM_TIME: str = "true"


def generate_launch_description() -> LaunchDescription:
    """Create the launch description for `robot_state_publisher`.

    Returns:
        Configured LaunchDescription declaring `use_sim_time` and launching
        the `robot_state_publisher` node with the compiled Xacro model.
    """
    package_share_dir: str = get_package_share_directory(DESCRIPTION_PACKAGE_NAME)
    xacro_file_path: str = os.path.join(
        package_share_dir, URDF_SUBDIRECTORY, ROOT_XACRO_FILENAME
    )

    declare_use_sim_time = DeclareLaunchArgument(
        name=USE_SIM_TIME_ARG,
        default_value=DEFAULT_USE_SIM_TIME,
        description="Use simulation clock published on /clock if true.",
    )

    use_sim_time_config = LaunchConfiguration(USE_SIM_TIME_ARG)
    robot_description_content = ParameterValue(
        Command(["xacro ", xacro_file_path]),
        value_type=str,
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description_content,
                "use_sim_time": use_sim_time_config,
            }
        ],
    )

    return LaunchDescription(
        [
            declare_use_sim_time,
            robot_state_publisher_node,
        ]
    )

