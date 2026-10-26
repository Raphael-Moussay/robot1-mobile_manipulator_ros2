# Mobile Manipulator Robot 1 - ROS 2 Simulation

Simulation and autonomous navigation workspace for a compact mobile manipulator competing in the **UTB Robogames "Robot Cleaner" (Robot uklízeč)** benchmark challenge.

Developed for **ROS 2 Jazzy Jalisco** and **Gazebo Harmonic (`ros_gz_sim`)**.

---

## 1. Package Overview: `robogames_arena_gazebo`

The `robogames_arena_gazebo` package provides the simulation environment for the competition arena:
- **Playing Surface**: 1.40 m (X) x 1.00 m (Y) static white plane centered at `(0, 0, 0)`.
- **LiDAR Perimeter Walls**: 4 black walls (30 mm thickness, 150 mm height) designed for 2D LiDAR feature detection and Nav2 costmap configuration.
- **Target Collection Box (Variant 2)**: Located in the top-right corner at `(X = +0.50 m, Y = +0.30 m)` rotated at 45° (`Yaw = -2.356 rad` facing the arena center). It features 3 front/side walls (100 mm height) and a rear rebound backboard (300 mm height).
- **Dynamic Duplo Bricks**: 6 Lego Duplo bricks (`0.064 x 0.032 x 0.020 m`, mass `0.015 kg`, high friction) spawned randomly across the board using Python and `ros_gz_sim create`.
- **Exclusion Zones**:
  - Wall perimeter safety margin: 10 cm clearance.
  - Robot starting area (bottom-left): $X < -0.35$ and $Y < -0.15$.
  - Target collection box area (top-right): $X > 0.25$ and $Y > 0.05$.
  - Inter-brick clearance: minimum 10 cm separation between any two bricks.

---

## 2. Technical Stack

| Component | Technology | Version |
| :--- | :--- | :--- |
| **Operating System** | Ubuntu Linux | 24.04 LTS (Noble) |
| **Middleware** | ROS 2 | Jazzy Jalisco |
| **Simulator** | Gazebo | Harmonic (`ros_gz_sim`) |
| **Build System** | colcon | `ament_python` |

---

## 3. Package Structure

```text
src/robogames_arena_gazebo/
├── launch/
│   └── start_competition.launch.py   # Spawns world in Gazebo Sim & 6 randomized bricks
├── models/
│   ├── duplo_brick/
│   │   ├── model.config               # Gazebo model configuration
│   │   └── model.sdf                  # Dynamic brick SDF (15g, exact inertia tensor, friction)
│   └── target_box/
│       ├── meshes/
│       │   ├── Box.mtl                # Material definition exported from Onshape
│       │   └── Box.obj                # CAD 3D mesh exported from Onshape
│       ├── model.config               # Gazebo target box model configuration
│       └── model.sdf                  # Target box SDF with visual & collision mesh
├── resource/
│   └── robogames_arena_gazebo         # ament_python resource index marker
├── robogames_arena_gazebo/
│   └── __init__.py                    # Python package module
├── package.xml                        # Package manifest & dependencies
├── setup.cfg                          # Installation script paths
├── setup.py                           # Recursive asset installer (os.walk)
└── worlds/
    └── cleaner_arena.sdf              # SDF world with physics, sun, ground, arena & box
```

---

## 4. Prerequisites & Installation

### Install ROS 2 Jazzy & Gazebo Harmonic Packages

Ensure ROS 2 Jazzy and the Gazebo Sim bridge dependencies are installed:

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-desktop \
  ros-jazzy-ros-gz-sim \
  ros-jazzy-ros-gz-bridge \
  python3-colcon-common-extensions
```

---

## 5. Build Instructions

Clone or open the repository, then build the workspace from the root directory:

```bash
# 1. Navigate to the workspace root
cd ~/Documents/Erasmus/robot1-mobile_manipulator_ros2

# 2. Source the underlying ROS 2 Jazzy environment
source /opt/ros/jazzy/setup.bash

# 3. Resolve and verify package dependencies
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# 4. Build the robogames_arena_gazebo package
colcon build --symlink-install --packages-select robogames_arena_gazebo
```

---

## 6. How to Run the Simulation

After a successful build, source the local overlay and execute the launch file:

```bash
# 1. Source the workspace overlay
source install/setup.bash

# 2. Launch the competition arena with randomized brick placement
ros2 launch robogames_arena_gazebo start_competition.launch.py
```

### Expected Behavior
1. Gazebo Harmonic starts automatically with physics running in real-time (`-r` mode).
2. The white competition field and perimeter black walls appear centered at `(0, 0, 0)`.
3. The target collection box is positioned in the upper-right corner at `(0.50, 0.30)`.
4. Six bright red Duplo bricks (`brick_1` through `brick_6`) drop onto the field in random locations outside the exclusion zones.

---

## 7. Useful Verification Commands

While the simulation is running in a terminal, inspect Gazebo entities and ROS topics in another terminal:

```bash
# Source environment
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# List Gazebo models
gz model --list

# List active ROS 2 nodes
ros2 node list
```

---

## 8. Next Development Steps

As outlined in [`plan.md`](plan.md):
- **Phase 1 (Sessions 1 to 3)**: Mobile base URDF/Xacro modeling, diff-drive controller, and LiDAR integration.
- **Phase 2 (Sessions 4 to 6)**: 2D SLAM mapping using `slam_toolbox`.
- **Phase 3 (Sessions 7 to 9)**: Nav2 stack integration (costmaps, AMCL, waypoints).
- **Phase 4 (Sessions 10 to 11)**: Arm manipulation and full competition mission execution.

