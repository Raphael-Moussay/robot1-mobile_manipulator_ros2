"""Convenience launch file starting keyboard teleoperation in a dedicated terminal."""

import shutil

from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import Node

BRINGUP_PACKAGE_NAME: str = "robot1_bringup"
TELEOP_EXECUTABLE_NAME: str = "keyboard_teleop"
XTERM_BINARY: str = "xterm"
XTERM_PREFIX_CMD: str = "xterm -title 'Robot 1 Keyboard Teleop' -e"


def generate_launch_description() -> LaunchDescription:
    """Launch `keyboard_teleop` inside `xterm` if available, else print guidance.

    Returns:
        Configured LaunchDescription starting a dedicated teleop terminal or
        logging the direct `ros2 run` command if `xterm` is not installed.
    """
    if shutil.which(XTERM_BINARY) is not None:
        return LaunchDescription(
            [
                Node(
                    package=BRINGUP_PACKAGE_NAME,
                    executable=TELEOP_EXECUTABLE_NAME,
                    name=TELEOP_EXECUTABLE_NAME,
                    output="screen",
                    prefix=XTERM_PREFIX_CMD,
                )
            ]
        )

    return LaunchDescription(
        [
            LogInfo(
                msg=(
                    "xterm is not installed; run keyboard teleoperation directly "
                    "in an interactive terminal with: "
                    "ros2 run robot1_bringup keyboard_teleop"
                )
            )
        ]
    )

