# my_mapper_turtlebot_pkg

`my_gui_turtlebot_pkg`에서 mapper 관련 파일만 분리한 패키지입니다.

## 실행
- 탐색 노드만 실행:
  - `ros2 launch my_mapper_turtlebot_pkg mapper_explorer.launch.py`
- stress maze 통합 실행:
  - `ros2 launch my_mapper_turtlebot_pkg stress_maze_explorer.launch.py`
- 분리 실행(권장 순서):
  1. `ros2 launch my_mapper_turtlebot_pkg stress_maze_world.launch.py use_sim_time:=true`
  2. `ros2 launch nav2_bringup bringup_launch.py use_sim_time:=true map:=/home/penguin/robot_ws/src/my_mapper_turtlebot_pkg/maps/stress_maze_map.yaml params_file:=/home/penguin/robot_ws/src/my_mapper_turtlebot_pkg/config/nav2_params_stress_maze.yaml autostart:=true use_composition:=False use_respawn:=False`
  3. `ros2 run my_mapper_turtlebot_pkg mapper_explorer`
  4. `rviz2 -d /home/penguin/robot_ws/src/my_mapper_turtlebot_pkg/rviz/explorer.rviz`

## 기본 튜닝값
- goal timeout: `45s`
- 정체 판정: `18s`
- obstacle/clearance 강화:
  - `max_obstacle_density = 0.24`
  - `min_clearance_radius_cells = 2`
- 병목 재진입 억제 강화:
  - `hard_blacklist_radius = 0.95m`
  - `hard_blacklist_ttl_sec = 120s`
  - `stalled_hotspot_trigger_count = 1`
