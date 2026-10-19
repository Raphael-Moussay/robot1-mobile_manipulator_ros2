# robogames_arena_gazebo

Gazebo Harmonic simulation package for the UTB Robogames "Robot Cleaner" competition arena under ROS 2 Jazzy.

## Quick Start

```bash
# From workspace root
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select robogames_arena_gazebo
source install/setup.bash
ros2 launch robogames_arena_gazebo start_competition.launch.py
```

See the main [README.md](../../README.md) for full architecture and technical documentation.
