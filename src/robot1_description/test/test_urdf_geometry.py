"""Unit tests validating Robot 1 URDF/Xacro geometry, mesh bounds, and Hokuyo laser height."""

import math
from pathlib import Path
import re
from typing import Dict, List, Set, Tuple
import unittest
import xml.etree.ElementTree as ET

PACKAGE_ROOT: Path = Path(__file__).resolve().parents[1]
URDF_DIR: Path = PACKAGE_ROOT / "urdf"
MESH_DIR: Path = PACKAGE_ROOT / "meshes" / "sensors"

DUPLO_BRICK_HEIGHT_M: float = 0.020
EXPECTED_LASER_HEIGHT_M: float = 0.017
EXPECTED_CAP_CLEARANCE_M: float = 0.002
FLOAT_TOLERANCE: float = 1e-6
EXPECTED_HOKUYO_BOUNDS: Tuple[
    Tuple[float, float], Tuple[float, float], Tuple[float, float]
] = (
    (-0.025, 0.025),
    (-0.025, 0.025),
    (0.000, 0.070),
)
MESH_TOLERANCE: float = 1e-4

EXPECTED_WHEEL_JOINTS: Tuple[str, ...] = (
    "front_left_wheel_joint",
    "rear_left_wheel_joint",
    "front_right_wheel_joint",
    "rear_right_wheel_joint",
)
EXPECTED_ARM_JOINTS: Tuple[str, ...] = (
    "Joint1",
    "Joint2",
    "Joint3",
    "Joint4",
    "Joint5",
)
EXPECTED_ALL_PUBLISHED_JOINTS: Tuple[str, ...] = (
    EXPECTED_WHEEL_JOINTS + EXPECTED_ARM_JOINTS
)
EXPECTED_ARM_MOUNT_X_M: float = 0.060


def evaluate_xacro_properties(xacro_paths: List[Path]) -> Dict[str, object]:
    """Parse and sequentially evaluate `<xacro:property>` expressions across files.

    Args:
        xacro_paths: Ordered list of Xacro XML files to inspect.

    Returns:
        Dictionary mapping property names to evaluated numeric or string values.
    """
    xacro_namespace: str = "{http://www.ros.org/wiki/xacro}property"
    context: Dict[str, object] = {
        "pi": math.pi,
        "True": True,
        "False": False,
        "inverted": True,
    }
    expression_pattern = re.compile(r"\$\{([^}]+)\}")

    for path in xacro_paths:
        tree = ET.parse(path)
        for elem in tree.getroot().iter(xacro_namespace):
            prop_name: str = elem.attrib.get("name", "")
            raw_value: str = elem.attrib.get("value", "")
            if not prop_name or not raw_value:
                continue

            match = expression_pattern.fullmatch(raw_value.strip())
            if match:
                context[prop_name] = eval(  # noqa: S307
                    match.group(1), {"__builtins__": {}}, context
                )
            else:
                try:
                    context[prop_name] = float(raw_value)
                except ValueError:
                    context[prop_name] = raw_value

    return context


class TestRobot1UrdfGeometry(unittest.TestCase):
    """Verify XML structure, mesh geometry, and Hokuyo URG-04LX optical clearance."""

    def test_all_xml_files_are_well_formed(self) -> None:
        """Ensure package.xml and every .xacro file parse without XML syntax errors."""
        xml_files: List[Path] = [PACKAGE_ROOT / "package.xml"] + sorted(
            URDF_DIR.rglob("*.xacro")
        )
        self.assertGreaterEqual(len(xml_files), 7)
        for xml_file in xml_files:
            with self.subTest(xml_file=str(xml_file)):
                tree = ET.parse(xml_file)
                self.assertIsNotNone(tree.getroot())

    def test_hokuyo_mesh_dimensions_and_materials(self) -> None:
        """Verify Hokuyo URG-04LX .obj bounding box matches 50x50x70 mm datasheet."""
        obj_path: Path = MESH_DIR / "hokuyo_urg04lx.obj"
        mtl_path: Path = MESH_DIR / "hokuyo_urg04lx.mtl"
        self.assertTrue(obj_path.is_file())
        self.assertTrue(mtl_path.is_file())

        defined_materials: Set[str] = set()
        for line in mtl_path.read_text(encoding="utf-8").splitlines():
            if line.startswith("newmtl "):
                defined_materials.add(line.split()[1])

        xs: List[float] = []
        ys: List[float] = []
        zs: List[float] = []
        used_materials: Set[str] = set()

        for line in obj_path.read_text(encoding="utf-8").splitlines():
            if line.startswith("v "):
                parts = line.split()
                xs.append(float(parts[1]))
                ys.append(float(parts[2]))
                zs.append(float(parts[3]))
            elif line.startswith("usemtl "):
                used_materials.add(line.split()[1])

        self.assertGreater(len(xs), 100)
        self.assertTrue(used_materials.issubset(defined_materials))

        actual_bounds = ((min(xs), max(xs)), (min(ys), max(ys)), (min(zs), max(zs)))
        for axis_actual, axis_expected in zip(actual_bounds, EXPECTED_HOKUYO_BOUNDS):
            self.assertAlmostEqual(
                axis_actual[0], axis_expected[0], delta=MESH_TOLERANCE
            )
            self.assertAlmostEqual(
                axis_actual[1], axis_expected[1], delta=MESH_TOLERANCE
            )

    def test_upside_down_hokuyo_detects_20mm_duplo_bricks(self) -> None:
        """Verify inverted Hokuyo beam sits at 17 mm (< 20 mm Duplo height) with 2 mm ground clearance."""
        ordered_files: List[Path] = [
            URDF_DIR / "sensors" / "hokuyo_urg04lx.xacro",
            URDF_DIR / "robot1_core.xacro",
            URDF_DIR / "gazebo_control.xacro",
            URDF_DIR / "robot1.urdf.xacro",
        ]
        props = evaluate_xacro_properties(ordered_files)

        chassis_ground_clearance = float(props["CHASSIS_GROUND_CLEARANCE"])
        hokuyo_height = float(props["HOKUYO_URG04LX_HEIGHT"])
        optical_z_offset = float(props["HOKUYO_URG04LX_OPTICAL_Z_OFFSET"])

        # Mounting face is flush under chassis_link (Z = CHASSIS_GROUND_CLEARANCE)
        # Inverted (Roll = +pi): local +Z points straight down toward floor (Z = 0)
        cap_height_above_floor: float = chassis_ground_clearance - hokuyo_height
        laser_plane_height_above_floor: float = (
            chassis_ground_clearance - optical_z_offset
        )

        self.assertAlmostEqual(
            cap_height_above_floor, EXPECTED_CAP_CLEARANCE_M, delta=FLOAT_TOLERANCE
        )
        self.assertGreater(cap_height_above_floor, 0.0)

        self.assertAlmostEqual(
            laser_plane_height_above_floor,
            EXPECTED_LASER_HEIGHT_M,
            delta=FLOAT_TOLERANCE,
        )
        self.assertLess(laser_plane_height_above_floor, DUPLO_BRICK_HEIGHT_M)
        self.assertGreater(laser_plane_height_above_floor, 0.0)

        # Verify REP-103 upright optical frame compensation cancels mounting Roll = +pi
        optical_roll_comp = float(props["optical_roll_compensation"])
        self.assertAlmostEqual(
            math.pi + optical_roll_comp, 0.0, delta=FLOAT_TOLERANCE
        )

        # Verify top-front manipulator arm base sits inside front half of chassis
        arm_mount_x = float(props["ARM_MOUNT_X_OFFSET"])
        arm_base_radius = float(props["ARM_BASE_RADIUS"])
        chassis_half_length = float(props["CHASSIS_LENGTH"]) / 2.0
        self.assertAlmostEqual(
            arm_mount_x, EXPECTED_ARM_MOUNT_X_M, delta=FLOAT_TOLERANCE
        )
        self.assertLessEqual(arm_mount_x + arm_base_radius, chassis_half_length)

    def test_gazebo_diff_drive_and_arm_joints_configured(self) -> None:
        """Verify Gazebo DiffDrive, JointStatePublisher, and 5-DOF arm controllers."""
        control_tree = ET.parse(URDF_DIR / "gazebo_control.xacro")
        root = control_tree.getroot()

        left_joints = [e.text for e in root.findall(".//left_joint")]
        right_joints = [e.text for e in root.findall(".//right_joint")]
        published_joints = [
            e.text
            for e in root.findall(".//plugin[@filename='gz-sim-joint-state-publisher-system']/joint_name")
        ]

        self.assertEqual(
            left_joints, ["front_left_wheel_joint", "rear_left_wheel_joint"]
        )
        self.assertEqual(
            right_joints, ["front_right_wheel_joint", "rear_right_wheel_joint"]
        )
        self.assertEqual(tuple(published_joints), EXPECTED_ALL_PUBLISHED_JOINTS)


if __name__ == "__main__":
    unittest.main()

