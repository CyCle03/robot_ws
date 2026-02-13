# Stress Maze World

Files:
- `stress_maze.world`: Gazebo world for stress testing
- `../maps/stress_maze_map.yaml`: matching static map

Quick start:
1. `ros2 launch my_gui_turtlebot_pkg stress_maze_world.launch.py`
2. `ros2 launch nav2_bringup bringup_launch.py use_sim_time:=True map:=/home/penguin/robot_ws/src/my_gui_turtlebot_pkg/maps/stress_maze_map.yaml`
3. `ros2 launch my_gui_turtlebot_pkg mapper_explorer.launch.py`

Notes:
- Keep `use_sim_time:=True` for all simulation nodes.
- Do not run SLAM and static-map localization together.
