"""Launch file for UTB Robogames Robot Cleaner competition arena.

This launch script launches Gazebo Harmonic (ros_gz_sim) with the competition world
and spawns 6 Lego Duplo bricks at collision-free, randomized positions.
"""

import math
import os
import random
from typing import List, Tuple

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

# Package and File Constants
PACKAGE_NAME: str = "robogames_arena_gazebo"
WORLD_FILE_NAME: str = "cleaner_arena.sdf"
WORLD_XACRO_NAME: str = "cleaner_arena.sdf.xacro"
MODEL_RELATIVE_PATH: str = os.path.join("models", "duplo_brick", "model.sdf")

# Spawning Constants
BRICK_COUNT: int = 6
BRICK_NAME_PREFIX: str = "brick_"
SPAWN_HEIGHT_Z: float = 0.025

# Arena Playable Surface Dimensions (meters)
ARENA_X_MIN: float = -0.70
ARENA_X_MAX: float = 0.70
ARENA_Y_MIN: float = -0.50
ARENA_Y_MAX: float = 0.50

# Safety Clearance Margins (meters)
WALL_SAFETY_MARGIN: float = 0.10
MIN_INTER_BRICK_DISTANCE: float = 0.10

# Valid Spawning Area Bounds (after wall margin clearance)
SPAWN_X_MIN: float = ARENA_X_MIN + WALL_SAFETY_MARGIN  # -0.60 m
SPAWN_X_MAX: float = ARENA_X_MAX - WALL_SAFETY_MARGIN  # +0.60 m
SPAWN_Y_MIN: float = ARENA_Y_MIN + WALL_SAFETY_MARGIN  # -0.40 m
SPAWN_Y_MAX: float = ARENA_Y_MAX - WALL_SAFETY_MARGIN  # +0.40 m

# Exclusion Zone 1: Robot Starting Area (Bottom-Left Corner)
ROBOT_EXCLUSION_X_MAX: float = -0.35
ROBOT_EXCLUSION_Y_MAX: float = -0.15

# Exclusion Zone 2: Target Zone (Top-Right Corner: X=+0.70m, Y=+0.50m)
# Official target zone radius: 0.30 m (Option 1 arc)
# Duplo spawning exclusion radius: 0.35 m (includes 5 cm safety clearance)
TARGET_CORNER_X: float = ARENA_X_MAX  # 0.70 m
TARGET_CORNER_Y: float = ARENA_Y_MAX  # 0.50 m
TARGET_EXCLUSION_RADIUS: float = 0.35  # 0.35 m exclusion arc from top-right corner

# Sampling Loop Limit
MAX_SAMPLING_ATTEMPTS: int = 2000


def is_in_robot_exclusion_zone(x_coord: float, y_coord: float) -> bool:
    """Check if the given coordinate lies inside the robot start area.

    Args:
        x_coord: The X position coordinate in meters.
        y_coord: The Y position coordinate in meters.

    Returns:
        True if the point lies inside the robot exclusion zone, False otherwise.
    """
    return x_coord < ROBOT_EXCLUSION_X_MAX and y_coord < ROBOT_EXCLUSION_Y_MAX


def is_in_target_zone_exclusion(x_coord: float, y_coord: float) -> bool:
    """Check if the given coordinate lies inside the 35cm target zone exclusion arc.

    Args:
        x_coord: The X position coordinate in meters.
        y_coord: The Y position coordinate in meters.

    Returns:
        True if the Euclidean distance from the top-right corner is <= TARGET_EXCLUSION_RADIUS.
    """
    distance_to_corner: float = math.hypot(
        x_coord - TARGET_CORNER_X, y_coord - TARGET_CORNER_Y
    )
    return distance_to_corner <= TARGET_EXCLUSION_RADIUS


def is_valid_brick_position(
    x_coord: float,
    y_coord: float,
    existing_positions: List[Tuple[float, float]],
) -> bool:
    """Validate whether candidate coordinates respect all boundaries and margins.

    Args:
        x_coord: Candidate X position in meters.
        y_coord: Candidate Y position in meters.
        existing_positions: List of previously placed (X, Y) positions.

    Returns:
        True if valid and collision-free, False otherwise.
    """
    if is_in_robot_exclusion_zone(x_coord, y_coord):
        return False

    if is_in_target_zone_exclusion(x_coord, y_coord):
        return False

    for prior_x, prior_y in existing_positions:
        distance: float = math.hypot(x_coord - prior_x, y_coord - prior_y)
        if distance < MIN_INTER_BRICK_DISTANCE:
            return False

    return True


def generate_random_brick_poses(
    count: int,
) -> List[Tuple[float, float, float]]:
    """Generate randomized 2D poses (X, Y, Yaw) adhering to competition zones.

    Args:
        count: Number of brick poses to generate.

    Returns:
        List of tuples containing (x, y, yaw_angle_radians).

    Raises:
        RuntimeError: If valid non-overlapping coordinates cannot be found within
            MAX_SAMPLING_ATTEMPTS iterations.
    """
    poses: List[Tuple[float, float, float]] = []
    positions: List[Tuple[float, float]] = []
    attempts: int = 0

    while len(poses) < count and attempts < MAX_SAMPLING_ATTEMPTS:
        attempts += 1
        candidate_x: float = random.uniform(SPAWN_X_MIN, SPAWN_X_MAX)
        candidate_y: float = random.uniform(SPAWN_Y_MIN, SPAWN_Y_MAX)

        if is_valid_brick_position(candidate_x, candidate_y, positions):
            candidate_yaw: float = random.uniform(-math.pi, math.pi)
            positions.append((candidate_x, candidate_y))
            poses.append((candidate_x, candidate_y, candidate_yaw))

    if len(poses) < count:
        raise RuntimeError(
            f"Failed to generate {count} valid brick positions after {attempts} attempts."
        )

    return poses


def compile_xacro_to_sdf(xacro_file_path: str, output_sdf_path: str) -> None:
    """Compile a Xacro world template into a standard SDFormat file on launch.

    Args:
        xacro_file_path: Absolute path to the source .sdf.xacro template.
        output_sdf_path: Absolute destination path for the generated .sdf file.
    """
    if not os.path.exists(xacro_file_path):
        return

    try:
        import xacro  # type: ignore

        doc = xacro.process_file(xacro_file_path)
        with open(output_sdf_path, "w", encoding="utf-8") as file_handle:
            file_handle.write(doc.toprettyxml(indent="  "))
    except Exception:
        import subprocess

        try:
            subprocess.run(
                ["xacro", xacro_file_path, "-o", output_sdf_path],
                check=True,
                capture_output=True,
            )
        except Exception:
            pass


def generate_launch_description() -> LaunchDescription:
    """Generate the launch description for the cleaner arena simulation.

    Returns:
        Configured LaunchDescription containing environment variables,
        Gazebo Sim instance, and brick spawner nodes.
    """
    package_share_directory: str = get_package_share_directory(PACKAGE_NAME)
    world_file_path: str = os.path.join(
        package_share_directory, "worlds", WORLD_FILE_NAME
    )
    xacro_file_path: str = os.path.join(
        package_share_directory, "worlds", WORLD_XACRO_NAME
    )
    # Automatically compile cleaner_arena.sdf from cleaner_arena.sdf.xacro if present
    compile_xacro_to_sdf(xacro_file_path, world_file_path)

    models_directory_path: str = os.path.join(package_share_directory, "models")
    brick_model_file_path: str = os.path.join(
        package_share_directory, MODEL_RELATIVE_PATH
    )

    # 1. Resource path environment variable configuration for Gazebo Sim
    gz_resource_env_models = AppendEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=models_directory_path,
    )
    parent_share_directory: str = os.path.dirname(package_share_directory)
    gz_resource_env_share = AppendEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=parent_share_directory,
    )

    # 2. Gazebo Sim launch description (started in running state '-r')
    ros_gz_sim_share: str = get_package_share_directory("ros_gz_sim")
    gz_sim_launch_script: str = os.path.join(
        ros_gz_sim_share, "launch", "gz_sim.launch.py"
    )

    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim_launch_script),
        launch_arguments={"gz_args": f"-r {world_file_path}"}.items(),
    )

    # 3. Dynamic spawner nodes for 6 randomized Duplo bricks
    generated_poses: List[Tuple[float, float, float]] = generate_random_brick_poses(
        BRICK_COUNT
    )

    spawner_nodes: List[Node] = []
    for index, (pos_x, pos_y, yaw_angle) in enumerate(generated_poses, start=1):
        brick_name: str = f"{BRICK_NAME_PREFIX}{index}"
        spawner_node = Node(
            package="ros_gz_sim",
            executable="create",
            output="screen",
            arguments=[
                "-file",
                brick_model_file_path,
                "-name",
                brick_name,
                "-x",
                f"{pos_x:.4f}",
                "-y",
                f"{pos_y:.4f}",
                "-z",
                f"{SPAWN_HEIGHT_Z:.4f}",
                "-Y",
                f"{yaw_angle:.4f}",
            ],
        )
        spawner_nodes.append(spawner_node)

    # Assemble launch actions
    launch_actions = [gz_resource_env_models, gz_resource_env_share, gazebo_sim]
    launch_actions.extend(spawner_nodes)

    return LaunchDescription(launch_actions)

