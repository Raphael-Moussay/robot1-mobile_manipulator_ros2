# Mobile Manipulator Simulation Project - Robot 1

## 1. Project Overview
This project aims to design, simulate, and program a compact mobile manipulator robot in a virtual environment. The system consists of a differential mobile base topped by a robotic arm with a maximum height of 45 cm. The entire development and testing process takes place in simulation using ROS 2.

## 2. Competition Research and Benchmark Selection
An analysis of existing robotics competitions (RoboCup@Home, RoboCup@Work, ARIAC, FIRST Tech Challenge, Eurobot) was conducted. 

Although these competitions offer valuable regulatory frameworks, none fully match the specific physical and functional constraints of a robot at this scale (a 45 cm arm on a compact base). 

Therefore, it was decided to adapt and create our own benchmark/competition in simulation. The robot will execute an autonomous course: navigate through an unknown environment, reach a target area, and perform a manipulation/demonstration task with its arm.

## 3. State of the Art: Simulation Software

| Simulator | Pros | Cons |
| :--- | :--- | :--- |
| **Gazebo (Classic / Sim)** | - Native and complete integration with ROS 2 and Nav2.<br>- Large ecosystem with many ready-to-use plugins (LiDAR, diff_drive).<br>- Recognized academic standard. | - Basic visual rendering and graphics.<br>- Fine-tuning friction parameters can be complex. |
| **Webots** | - Quick setup and lightweight installation.<br>- Intuitive user interface and easy environment creation. | - ROS 2 ecosystem slightly less documented than Gazebo's.<br>- Fewer pre-integrated community packages. |
| **NVIDIA Isaac Sim** | - Exceptional photorealistic rendering (Omniverse engine).<br>- High-performance execution for reinforcement learning. | - Requires a very powerful hardware setup (NVIDIA RTX GPU).<br>- Steep learning curve for a short project. |

## 4. Work Plan (11 Sessions)

### Phase 1: Mobile Base & Teleoperation (Sessions 1 to 3)
- **Session 1:** URDF/Xacro modeling of the mobile chassis, wheel adjustments, and assembly of the robotic arm.
- **Session 2:** Gazebo integration and configuration of differential drive (`diff_drive`) and LiDAR plugins.
- **Session 3:** Setup and validation of keyboard teleoperation (`teleop_twist_keyboard`).

### Phase 2: Mapping & SLAM (Sessions 4 to 6)
- **Session 4:** Creation of the Gazebo virtual world (walls, obstacles, and intervention areas).
- **Session 5:** Real-time environment mapping using `slam_toolbox`.
- **Session 6:** Saving and exporting the generated map (`.yaml` / `.pgm`).

### Phase 3: Autonomous Navigation with Nav2 (Sessions 7 to 9)
- **Session 7:** Configuration of the Nav2 stack (global/local costmaps, AMCL).
- **Session 8:** Validation of target point navigation (*2D Goal Pose*) in RViz.
- **Session 9:** Implementation of waypoint following.

### Phase 4: Competition Scenario & Deliverables (Sessions 10 and 11)
- **Session 10:** Execution of the full competition scenario (autonomous navigation to the intervention area + triggering arm movements).
- **Session 11:** Recording video demonstrations, extracting node graphs (`rqt_graph`), checking transform trees (`tf2_tools`), and finalizing the report.