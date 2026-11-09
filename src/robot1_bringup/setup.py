"""Setuptools configuration for the robot1_bringup ROS 2 package."""

import os
from typing import Dict, List, Tuple

from setuptools import find_packages, setup

PACKAGE_NAME: str = "robot1_bringup"
PACKAGE_VERSION: str = "0.1.0"
ASSET_DIRECTORIES: Tuple[str, ...] = ("launch", "config")
EXCLUDED_DIR_NAMES: Tuple[str, ...] = ("__pycache__",)
EXCLUDED_EXTENSIONS: Tuple[str, ...] = (".pyc", ".pyo")


def collect_data_files(
    package_name: str,
    asset_directories: Tuple[str, ...],
) -> List[Tuple[str, List[str]]]:
    """Recursively collect package data files while preserving directory structure.

    Args:
        package_name: Name of the target ROS 2 package.
        asset_directories: Relative directory roots to install under share/.

    Returns:
        List of (install_destination, source_file_list) tuples for setuptools.
    """
    install_mapping: Dict[str, List[str]] = {}

    for directory in asset_directories:
        if not os.path.isdir(directory):
            continue
        for root_dir, subdirs, filenames in os.walk(directory):
            subdirs[:] = [d for d in subdirs if d not in EXCLUDED_DIR_NAMES]
            valid_files: List[str] = [
                os.path.join(root_dir, filename)
                for filename in sorted(filenames)
                if not filename.endswith(EXCLUDED_EXTENSIONS)
            ]
            if valid_files:
                destination: str = os.path.join("share", package_name, root_dir)
                install_mapping[destination] = valid_files

    base_data_files: List[Tuple[str, List[str]]] = [
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ]
    base_data_files.extend(sorted(install_mapping.items()))
    return base_data_files


setup(
    name=PACKAGE_NAME,
    version=PACKAGE_VERSION,
    packages=find_packages(exclude=["test"]),
    data_files=collect_data_files(PACKAGE_NAME, ASSET_DIRECTORIES),
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Raphael Moussay",
    maintainer_email="raphael.moussay@student.utb.cz",
    description="Simulation bringup, ROS-Gazebo bridge, and keyboard teleoperation for Robot 1.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "keyboard_teleop = robot1_bringup.keyboard_teleop:main",
            "laser_self_filter = robot1_bringup.laser_self_filter:main",
        ],
    },
)

