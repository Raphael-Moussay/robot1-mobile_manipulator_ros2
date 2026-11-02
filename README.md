# Mobile Manipulator Robot 1 - ROS 2 Simulation

Simulation and autonomous navigation workspace for a compact mobile manipulator competing in the **UTB Robogames "Robot Cleaner" (Robot uklízeč)** benchmark challenge.

Developed for **ROS 2 Jazzy Jalisco** and **Gazebo Harmonic (`ros_gz_sim`)**.

---

## 1. Workspace Architecture

Following standard ROS 2 separation of concerns, the workspace is organized into three decoupled packages:

| Package | Role |
| :--- | :--- |
| **`robogames_arena_gazebo`** | Competition world (`1.40 m x 1.00 m`), 10 mm flush perimeter borders, 30 cm target zone arc, Onshape target collection box, and randomized spawning of 6 Lego Duplo bricks (`64 x 32 x 20 mm`). |
| **`robot1_description`** | Hardware-agnostic modular URDF/Xacro model of Robot 1 (4-wheel rectangular cuboid base + upside-down **Hokuyo URG-04LX** 2D LiDAR mesh & Gazebo Harmonic `gpu_lidar` macro) and RViz2 configuration. |
| **`robot1_bringup`** | End-to-end simulation launch orchestration, `ros_gz_bridge` YAML topic mapping (`/clock`, `/cmd_vel`, `/odom`, `/tf`, `/joint_states`, `/scan`), and interactive keyboard teleoperation (`keyboard_teleop`). |

### Hokuyo URG-04LX Upside-Down Mounting Geometry
According to official Hokuyo datasheet specifications (Drawing No. `C-40-3362`), the URG-04LX housing measures `50 x 50 x 70 mm` (`160 g`) with its optical scanning plane located `55 mm` from the base (`15 mm` from the top cap).
- Because Lego Duplo bricks are only **`20 mm` tall**, an upright floor-level mount (`>= 57 mm`) would shoot above the bricks.
- Mounting the URG-04LX **upside-down (`Roll = pi rad`) underneath the rectangular chassis** (`72 mm` ground clearance) places the sensor cap `2 mm` above the playing surface and positions the horizontal laser plane at **`17 mm` above the floor**, cleanly detecting all `20 mm` Duplo bricks across the arena while keeping `hokuyo_laser_frame` upright (`+Z` up, REP-103 compliant).

---

## 2. Technical Stack

| Component | Technology | Version |
| :--- | :--- | :--- |
| **Operating System** | Ubuntu Linux | 24.04 LTS (Noble) |
| **Middleware** | ROS 2 | Jazzy Jalisco |
| **Simulator** | Gazebo | Harmonic (`ros_gz_sim`) |
| **Build System** | colcon | `ament_python` |

---

## 3. Repository Structure

```text
src/
├── robogames_arena_gazebo/
│   ├── launch/start_competition.launch.py
│   ├── models/
│   │   ├── duplo_brick/
│   │   └── target_box/
│   └── worlds/cleaner_arena.sdf.xacro
├── robot1_description/
│   ├── launch/description.launch.py
│   ├── meshes/sensors/hokuyo_urg04lx.{obj,mtl}
│   ├── rviz/robot1_lidar_test.rviz
│   └── urdf/
│       ├── robot1.urdf.xacro
│       ├── robot1_core.xacro
│       ├── gazebo_control.xacro
│       ├── inertial_macros.xacro
│       ├── materials.xacro
│       └── sensors/hokuyo_urg04lx.xacro
└── robot1_bringup/
    ├── config/ros_gz_bridge.yaml
    ├── launch/
    │   ├── sim_arena_teleop.launch.py
    │   └── teleop_keyboard.launch.py
    └── robot1_bringup/keyboard_teleop.py
```

---

## 4. Prerequisites & Build Instructions

Install ROS 2 Jazzy and Gazebo Harmonic dependencies:

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-desktop \
  ros-jazzy-ros-gz-sim \
  ros-jazzy-ros-gz-bridge \
  ros-jazzy-xacro \
  ros-jazzy-teleop-twist-keyboard \
  python3-colcon-common-extensions
```

Build the workspace from the repository root:

```bash
cd ~/Documents/Erasmus/robot1-mobile_manipulator_ros2
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

---

## 5. Running the Full Simulation & Keyboard Teleoperation

### Terminal 1: Launch Arena + 6 Duplo Bricks + Robot 1 + LiDAR Bridge + RViz2

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch robot1_bringup sim_arena_teleop.launch.py
```

*(Optional: pass `use_rviz:=false` to disable automatic RViz2 startup.)*

### Terminal 2: Drive Robot 1 with the Keyboard

Run the dedicated arena-tuned keyboard teleoperation node (supports **Arrow keys**, French **AZERTY `Z/Q/S/D`**, and **QWERTY `W/A/S/D`**, plus `Space` to stop and `+`/`-` to adjust speed):

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run robot1_bringup keyboard_teleop
```

Alternatively, standard `teleop_twist_keyboard` is also directly compatible:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
