"""Robot footprint self-filter node for the Hokuyo URG-04LX 2D LiDAR.

Subscribes to `/scan_raw` from Gazebo, removes laser rays that intersect
Robot 1's own front wheels or chassis bounding box by setting their range to
`NaN`, and republishes the clean scan on `/scan` for RViz2 and Nav2.
"""

import math
from typing import List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import LaserScan

# Node & Topic Constants
NODE_NAME: str = "laser_self_filter"
RAW_SCAN_TOPIC: str = "/scan_raw"
FILTERED_SCAN_TOPIC: str = "/scan"
PUBLISHER_QUEUE_DEPTH: int = 10

# Hokuyo URG-04LX Pose Relative to base_footprint (from robot1_core.xacro)
# CHASSIS_LENGTH / 2.0 (0.125) - HOKUYO_DEPTH / 2.0 (0.025) - INSET (0.002) = 0.098 m
LIDAR_X_OFFSET_FROM_BASE_M: float = 0.098
LIDAR_Y_OFFSET_FROM_BASE_M: float = 0.000

# Robot Self-Occlusion Bounding Box in base_footprint Coordinates (Meters)
# Chassis spans X in [-0.125, +0.125]; wheels span X in [-0.135, +0.080], |Y| in [0.083, 0.105]
SELF_BOX_X_MIN_M: float = -0.145
SELF_BOX_X_MAX_M: float = 0.130
SELF_BOX_HALF_WIDTH_Y_M: float = 0.118


def is_point_inside_robot_footprint(
    range_m: float,
    angle_rad: float,
    range_min_m: float,
    range_max_m: float,
) -> bool:
    """Check whether a polar LiDAR measurement hits the robot's own body or wheels.

    Args:
        range_m: Measured radial distance in meters.
        angle_rad: Ray angle in `hokuyo_laser_frame` in radians.
        range_min_m: Minimum valid sensor range in meters.
        range_max_m: Maximum valid sensor range in meters.

    Returns:
        True if the Cartesian hit point lies inside the robot's bounding box,
        False otherwise.
    """
    if not math.isfinite(range_m):
        return False
    if range_m < range_min_m or range_m > range_max_m:
        return False

    hit_x_base: float = LIDAR_X_OFFSET_FROM_BASE_M + range_m * math.cos(angle_rad)
    hit_y_base: float = LIDAR_Y_OFFSET_FROM_BASE_M + range_m * math.sin(angle_rad)

    within_x: bool = SELF_BOX_X_MIN_M <= hit_x_base <= SELF_BOX_X_MAX_M
    within_y: bool = abs(hit_y_base) <= SELF_BOX_HALF_WIDTH_Y_M
    return within_x and within_y


def filter_scan_ranges(
    ranges: List[float],
    angle_min_rad: float,
    angle_increment_rad: float,
    range_min_m: float,
    range_max_m: float,
) -> List[float]:
    """Return a new range list with self-occluded wheel hits replaced by NaN.

    Args:
        ranges: Raw radial distance measurements in meters.
        angle_min_rad: Starting scan angle in radians.
        angle_increment_rad: Angular step between consecutive rays in radians.
        range_min_m: Sensor minimum range in meters.
        range_max_m: Sensor maximum range in meters.

    Returns:
        New list of ranges where hits inside the robot footprint are `math.nan`.
    """
    filtered_ranges: List[float] = []
    for ray_index, distance_m in enumerate(ranges):
        ray_angle_rad: float = (
            angle_min_rad + float(ray_index) * angle_increment_rad
        )
        if is_point_inside_robot_footprint(
            distance_m, ray_angle_rad, range_min_m, range_max_m
        ):
            filtered_ranges.append(math.nan)
        else:
            filtered_ranges.append(distance_m)
    return filtered_ranges


class LaserSelfFilterNode(Node):
    """ROS 2 node filtering robot wheel hits from `/scan_raw` into `/scan`."""

    def __init__(self) -> None:
        """Initialize the `/scan_raw` subscriber and `/scan` publisher."""
        super().__init__(NODE_NAME)

        # RELIABLE publisher is compatible with both RELIABLE and BEST_EFFORT subscribers
        reliable_scan_qos = QoSProfile(
            depth=PUBLISHER_QUEUE_DEPTH,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self._scan_publisher = self.create_publisher(
            LaserScan,
            FILTERED_SCAN_TOPIC,
            reliable_scan_qos,
        )
        self._scan_subscription = self.create_subscription(
            LaserScan,
            RAW_SCAN_TOPIC,
            self._on_raw_scan_received,
            qos_profile_sensor_data,
        )

    def _on_raw_scan_received(self, raw_msg: LaserScan) -> None:
        """Filter incoming `LaserScan` and publish the cleaned scan on `/scan`.

        Args:
            raw_msg: Incoming `sensor_msgs/msg/LaserScan` from Gazebo.
        """
        filtered_msg = LaserScan()
        filtered_msg.header = raw_msg.header
        filtered_msg.angle_min = raw_msg.angle_min
        filtered_msg.angle_max = raw_msg.angle_max
        filtered_msg.angle_increment = raw_msg.angle_increment
        filtered_msg.time_increment = raw_msg.time_increment
        filtered_msg.scan_time = raw_msg.scan_time
        filtered_msg.range_min = raw_msg.range_min
        filtered_msg.range_max = raw_msg.range_max
        filtered_msg.ranges = filter_scan_ranges(
            list(raw_msg.ranges),
            raw_msg.angle_min,
            raw_msg.angle_increment,
            raw_msg.range_min,
            raw_msg.range_max,
        )
        filtered_msg.intensities = list(raw_msg.intensities)
        self._scan_publisher.publish(filtered_msg)


def main(args: Optional[List[str]] = None) -> None:
    """Entry point for the `laser_self_filter` executable.

    Args:
        args: Optional CLI arguments forwarded to `rclpy.init`.
    """
    rclpy.init(args=args)
    filter_node = LaserSelfFilterNode()
    try:
        rclpy.spin(filter_node)
    except KeyboardInterrupt:
        pass
    finally:
        filter_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
