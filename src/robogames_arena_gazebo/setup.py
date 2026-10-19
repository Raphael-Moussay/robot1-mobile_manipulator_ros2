"""Setup script for robogames_arena_gazebo ROS 2 package."""

import os
from glob import glob
from typing import List, Tuple
from setuptools import find_packages, setup

PACKAGE_NAME: str = "robogames_arena_gazebo"


def collect_data_files(
    directories: List[str],
) -> List[Tuple[str, List[str]]]:
    """Collect all files recursively from source directories for data_files installation.

    Args:
        directories: List of directory names relative to the package root.

    Returns:
        List of tuples mapping destination directory paths to source file lists.
    """
    collected_files: List[Tuple[str, List[str]]] = []

    for source_dir in directories:
        for root_dir, _sub_dirs, filenames in os.walk(source_dir):
            if "__pycache__" in root_dir:
                continue
            valid_filenames: List[str] = [
                filename
                for filename in filenames
                if not filename.startswith(".") and not filename.endswith(".pyc")
            ]
            if valid_filenames:
                file_paths: List[str] = [
                    os.path.join(root_dir, filename) for filename in valid_filenames
                ]
                destination_dir: str = os.path.join(
                    "share", PACKAGE_NAME, root_dir
                )
                collected_files.append((destination_dir, file_paths))

    return collected_files


RESOURCE_INDEX_DESTINATION: str = os.path.join(
    "share", "ament_index", "resource_index", "packages"
)
PACKAGE_SHARE_DESTINATION: str = os.path.join("share", PACKAGE_NAME)

DATA_FILES: List[Tuple[str, List[str]]] = [
    (RESOURCE_INDEX_DESTINATION, [os.path.join("resource", PACKAGE_NAME)]),
    (PACKAGE_SHARE_DESTINATION, ["package.xml"]),
]

# Recursively collect launch, worlds, and models asset directories
TARGET_DIRECTORIES: List[str] = ["launch", "worlds", "models"]
DATA_FILES.extend(collect_data_files(TARGET_DIRECTORIES))

setup(
    name=PACKAGE_NAME,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=DATA_FILES,
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Robotics Engineer",
    maintainer_email="user@todo.todo",
    description="Gazebo Harmonic simulation package for UTB Robogames Robot Cleaner competition arena.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [],
    },
)
