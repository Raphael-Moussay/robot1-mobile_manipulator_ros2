"""Unit tests for Robot 1 bringup bridge configuration, spawn geometry, and teleop logic."""

import importlib.util
import math
from pathlib import Path
import sys
import types
from typing import Dict, List, Optional, Set, Tuple
import unittest
import xml.etree.ElementTree as ET

import yaml

PACKAGE_ROOT: Path = Path(__file__).resolve().parents[1]
WORKSPACE_SRC_ROOT: Path = PACKAGE_ROOT.parent
BRIDGE_YAML_PATH: Path = PACKAGE_ROOT / "config" / "ros_gz_bridge.yaml"
NAV2_YAML_PATH: Path = PACKAGE_ROOT / "config" / "nav2_params.yaml"
ARENA_WORLD_SDF_PATH: Path = (
    WORKSPACE_SRC_ROOT
    / "robogames_arena_gazebo"
    / "worlds"
    / "cleaner_arena.sdf"
)
TELEOP_MODULE_PATH: Path = (
    PACKAGE_ROOT / "robot1_bringup" / "keyboard_teleop.py"
)
SELF_FILTER_MODULE_PATH: Path = (
    PACKAGE_ROOT / "robot1_bringup" / "laser_self_filter.py"
)

EXPECTED_BRIDGED_TOPICS: Dict[str, Tuple[str, str, str]] = {
    "/clock": ("rosgraph_msgs/msg/Clock", "gz.msgs.Clock", "GZ_TO_ROS"),
    "/cmd_vel": ("geometry_msgs/msg/Twist", "gz.msgs.Twist", "ROS_TO_GZ"),
    "/odom": ("nav_msgs/msg/Odometry", "gz.msgs.Odometry", "GZ_TO_ROS"),
    "/tf": ("tf2_msgs/msg/TFMessage", "gz.msgs.Pose_V", "GZ_TO_ROS"),
    "/joint_states": (
        "sensor_msgs/msg/JointState",
        "gz.msgs.Model",
        "GZ_TO_ROS",
    ),
    "/scan_raw": (
        "sensor_msgs/msg/LaserScan",
        "gz.msgs.LaserScan",
        "GZ_TO_ROS",
    ),
}

# Arena start zone bounds from start_competition.launch.py
START_ZONE_X_MIN: float = -0.70
START_ZONE_X_MAX: float = -0.35
START_ZONE_Y_MIN: float = -0.50
START_ZONE_Y_MAX: float = -0.15

# Robot outer bounding box half-extents (0.250 m length x 0.210 m outer tire width)
ROBOT_HALF_LENGTH_M: float = 0.125
ROBOT_HALF_WIDTH_M: float = 0.105
SPAWN_X_M: float = -0.525
SPAWN_Y_M: float = -0.325
SPAWN_YAW_RAD: float = 0.62
MIN_OS_AUTOREPEAT_DELAY_SEC: float = 0.50


def load_teleop_module_with_stubs() -> types.ModuleType:
    """Import `keyboard_teleop.py` using lightweight ROS 2 stubs when offline.

    Returns:
        Loaded `keyboard_teleop` Python module object.
    """
    saved_modules: Dict[str, Optional[types.ModuleType]] = {}
    stub_names = ("rclpy", "rclpy.node", "geometry_msgs", "geometry_msgs.msg")
    for mod_name in stub_names:
        saved_modules[mod_name] = sys.modules.get(mod_name)

    try:
        if "rclpy" not in sys.modules:
            rclpy_stub = types.ModuleType("rclpy")
            rclpy_node_stub = types.ModuleType("rclpy.node")

            class StubNode:
                """Minimal stub for `rclpy.node.Node`."""

                def __init__(self, node_name: str) -> None:
                    self.node_name = node_name

            rclpy_node_stub.Node = StubNode  # type: ignore[attr-defined]
            sys.modules["rclpy"] = rclpy_stub
            sys.modules["rclpy.node"] = rclpy_node_stub

        if "geometry_msgs.msg" not in sys.modules:
            geom_pkg = types.ModuleType("geometry_msgs")
            geom_msg = types.ModuleType("geometry_msgs.msg")

            class StubTwist:
                """Minimal stub for `geometry_msgs.msg.Twist`."""

            geom_msg.Twist = StubTwist  # type: ignore[attr-defined]
            sys.modules["geometry_msgs"] = geom_pkg
            sys.modules["geometry_msgs.msg"] = geom_msg

        spec = importlib.util.spec_from_file_location(
            "keyboard_teleop_under_test", TELEOP_MODULE_PATH
        )
        assert spec is not None and spec.loader is not None
        module = importlib.util.module_from_spec(spec)
        sys.modules[spec.name] = module
        spec.loader.exec_module(module)
        return module
    finally:
        for mod_name, original in saved_modules.items():
            if original is None:
                sys.modules.pop(mod_name, None)
            else:
                sys.modules[mod_name] = original


class TestRobot1Bringup(unittest.TestCase):
    """Validate bridge YAML mappings, world sensor system, spawn bounds, and teleop bindings."""

    def test_ros_gz_bridge_yaml_completeness(self) -> None:
        """Ensure all 6 required simulation topics are bridged with valid types."""
        entries: List[Dict[str, str]] = yaml.safe_load(
            BRIDGE_YAML_PATH.read_text(encoding="utf-8")
        )
        self.assertIsInstance(entries, list)

        found_topics: Dict[str, Tuple[str, str, str]] = {}
        for item in entries:
            found_topics[item["ros_topic_name"]] = (
                item["ros_type_name"],
                item["gz_type_name"],
                item["direction"],
            )

        self.assertEqual(found_topics, EXPECTED_BRIDGED_TOPICS)

    def test_arena_world_has_ogre2_sensors_plugin_and_untouched_target_box(self) -> None:
        """Ensure cleaner_arena.sdf loads gz-sim-sensors-system and preserves target_box pose."""
        tree = ET.parse(ARENA_WORLD_SDF_PATH)
        root = tree.getroot()

        sensor_plugins = [
            p
            for p in root.findall(".//plugin")
            if p.attrib.get("filename") == "gz-sim-sensors-system"
        ]
        self.assertEqual(len(sensor_plugins), 1)
        render_engine = sensor_plugins[0].findtext("render_engine")
        self.assertEqual(render_engine, "ogre2")

        # Verify user's custom target_box pose was preserved untouched
        includes = root.findall(".//include")
        target_poses = [
            inc.findtext("pose")
            for inc in includes
            if inc.findtext("name") == "target_box"
        ]
        self.assertEqual(target_poses, ["0.70 0.50 0.01 0 0 -2.356194"])

    def test_robot_spawn_fits_strictly_inside_start_zone(self) -> None:
        """Verify all 4 rotated corners of Robot 1 at spawn remain inside the start zone."""
        cos_yaw: float = math.cos(SPAWN_YAW_RAD)
        sin_yaw: float = math.sin(SPAWN_YAW_RAD)

        local_corners: List[Tuple[float, float]] = [
            (ROBOT_HALF_LENGTH_M, ROBOT_HALF_WIDTH_M),
            (ROBOT_HALF_LENGTH_M, -ROBOT_HALF_WIDTH_M),
            (-ROBOT_HALF_LENGTH_M, ROBOT_HALF_WIDTH_M),
            (-ROBOT_HALF_LENGTH_M, -ROBOT_HALF_WIDTH_M),
        ]

        for local_x, local_y in local_corners:
            world_x: float = SPAWN_X_M + local_x * cos_yaw - local_y * sin_yaw
            world_y: float = SPAWN_Y_M + local_x * sin_yaw + local_y * cos_yaw
            self.assertGreater(world_x, START_ZONE_X_MIN)
            self.assertLess(world_x, START_ZONE_X_MAX)
            self.assertGreater(world_y, START_ZONE_Y_MIN)
            self.assertLess(world_y, START_ZONE_Y_MAX)

    def test_keyboard_teleop_bindings_and_clamping(self) -> None:
        """Verify Arrow, AZERTY, and QWERTY key bindings and speed clamping."""
        teleop = load_teleop_module_with_stubs()

        self.assertAlmostEqual(teleop.clamp_value(0.01, 0.05, 0.75), 0.05)
        self.assertAlmostEqual(teleop.clamp_value(1.20, 0.05, 0.75), 0.75)
        self.assertAlmostEqual(teleop.clamp_value(0.25, 0.05, 0.75), 0.25)

        bindings = teleop.KEY_DIRECTION_BINDINGS
        required_keys: Set[str] = {
            "ARROW_UP",
            "ARROW_DOWN",
            "ARROW_LEFT",
            "ARROW_RIGHT",
            "z",
            "q",
            "s",
            "d",
            "w",
            "a",
        }
        self.assertTrue(required_keys.issubset(set(bindings.keys())))
        self.assertEqual(bindings["z"].linear_sign, 1.0)
        self.assertEqual(bindings["w"].linear_sign, 1.0)
        self.assertEqual(bindings["s"].linear_sign, -1.0)
        self.assertEqual(bindings["q"].angular_sign, 1.0)
        self.assertEqual(bindings["a"].angular_sign, 1.0)
        self.assertEqual(bindings["d"].angular_sign, -1.0)
        self.assertGreater(
            teleop.DEADMAN_WATCHDOG_TIMEOUT_SEC, MIN_OS_AUTOREPEAT_DELAY_SEC
        )

    def test_laser_self_filter_removes_front_wheels_and_keeps_duplos(self) -> None:
        """Verify front wheel hits are replaced by NaN while external Duplo hits remain intact."""
        saved_modules: Dict[str, Optional[types.ModuleType]] = {}
        stub_names = (
            "rclpy",
            "rclpy.node",
            "rclpy.qos",
            "sensor_msgs",
            "sensor_msgs.msg",
        )
        for mod_name in stub_names:
            saved_modules[mod_name] = sys.modules.get(mod_name)

        try:
            rclpy_stub = types.ModuleType("rclpy")
            rclpy_node_stub = types.ModuleType("rclpy.node")
            rclpy_qos_stub = types.ModuleType("rclpy.qos")
            sensor_pkg = types.ModuleType("sensor_msgs")
            sensor_msg = types.ModuleType("sensor_msgs.msg")

            rclpy_node_stub.Node = object  # type: ignore[attr-defined]
            rclpy_qos_stub.DurabilityPolicy = object  # type: ignore[attr-defined]
            rclpy_qos_stub.QoSProfile = object  # type: ignore[attr-defined]
            rclpy_qos_stub.ReliabilityPolicy = object  # type: ignore[attr-defined]
            rclpy_qos_stub.qos_profile_sensor_data = object()  # type: ignore[attr-defined]
            sensor_msg.LaserScan = object  # type: ignore[attr-defined]

            sys.modules["rclpy"] = rclpy_stub
            sys.modules["rclpy.node"] = rclpy_node_stub
            sys.modules["rclpy.qos"] = rclpy_qos_stub
            sys.modules["sensor_msgs"] = sensor_pkg
            sys.modules["sensor_msgs.msg"] = sensor_msg

            spec = importlib.util.spec_from_file_location(
                "laser_self_filter_under_test", SELF_FILTER_MODULE_PATH
            )
            assert spec is not None and spec.loader is not None
            self_filter = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(self_filter)
        finally:
            for mod_name, original in saved_modules.items():
                if original is None:
                    sys.modules.pop(mod_name, None)
                else:
                    sys.modules[mod_name] = original

        # Ray at +112 deg (1.954 rad) hitting left front wheel at r = 0.095 m
        self.assertTrue(
            self_filter.is_point_inside_robot_footprint(0.095, 1.954, 0.05, 4.0)
        )
        # Ray at 0 deg hitting a Duplo brick 6 cm ahead of LiDAR (X_base = +0.158 m)
        self.assertFalse(
            self_filter.is_point_inside_robot_footprint(0.060, 0.0, 0.05, 4.0)
        )

        filtered = self_filter.filter_scan_ranges(
            [0.095, 0.250], 1.954, -1.954, 0.05, 4.0
        )
        self.assertTrue(math.isnan(filtered[0]))
        self.assertAlmostEqual(filtered[1], 0.250)

    def test_mapless_nav2_params_configuration(self) -> None:
        """Ensure nav2_params.yaml configures mapless rolling-window navigation in odom."""
        nav2_cfg = yaml.safe_load(NAV2_YAML_PATH.read_text(encoding="utf-8"))
        local_cm = nav2_cfg["local_costmap"]["local_costmap"]["ros__parameters"]
        global_cm = nav2_cfg["global_costmap"]["global_costmap"]["ros__parameters"]

        self.assertTrue(local_cm["rolling_window"])
        self.assertTrue(global_cm["rolling_window"])
        self.assertEqual(local_cm["global_frame"], "odom")
        self.assertEqual(global_cm["global_frame"], "odom")
        self.assertNotIn("static_layer", global_cm["plugins"])
        self.assertFalse(
            nav2_cfg["controller_server"]["ros__parameters"]["enable_stamped_cmd_vel"]
        )


if __name__ == "__main__":
    unittest.main()
