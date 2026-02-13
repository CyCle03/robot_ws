# Stress Maze World

Files:
- `stress_maze.world`: Gazebo world for stress testing
- `../maps/stress_maze_map.yaml`: matching static map

Quick start:
1. `ros2 launch my_mapper_turtlebot_pkg stress_maze_world.launch.py`
2. `ros2 launch nav2_bringup bringup_launch.py use_sim_time:=true map:=/home/penguin/robot_ws/src/my_mapper_turtlebot_pkg/maps/stress_maze_map.yaml params_file:=/home/penguin/robot_ws/src/my_mapper_turtlebot_pkg/config/nav2_params_stress_maze.yaml autostart:=true use_composition:=False use_respawn:=False`
3. `ros2 run my_mapper_turtlebot_pkg mapper_explorer`
4. `rviz2 -d /home/penguin/robot_ws/src/my_mapper_turtlebot_pkg/rviz/explorer.rviz`

Notes:
- Keep `use_sim_time:=True` for all simulation nodes.
- Do not run SLAM and static-map localization together.
