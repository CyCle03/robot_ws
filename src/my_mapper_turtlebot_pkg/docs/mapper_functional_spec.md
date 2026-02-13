# 기능명세서 (my_mapper_turtlebot_pkg)

## 1. 목적
- Frontier 탐색 관련 기능(`mapper_explorer`, `mapper_frontier_utils`)을 GUI/Patrol/Obstacle Avoid 패키지에서 분리해 독립 배포한다.

## 2. 포함 파일
- 노드: `my_mapper_turtlebot_pkg/mapper_explorer.py`
- 유틸: `my_mapper_turtlebot_pkg/mapper_frontier_utils.py`
- launch: `launch/mapper_explorer.launch.py`, `launch/stress_maze_explorer.launch.py`
- RViz 설정: `rviz/explorer.rviz`
- Nav2 파라미터: `config/nav2_params_stress_maze.yaml`

## 3. 실행
- 탐색 노드만 실행:
  - `ros2 launch my_mapper_turtlebot_pkg mapper_explorer.launch.py`
- stress maze 통합 실행:
  - `ros2 launch my_mapper_turtlebot_pkg stress_maze_explorer.launch.py`
- 분리 실행:
  1. `ros2 launch my_mapper_turtlebot_pkg stress_maze_world.launch.py use_sim_time:=true`
  2. `ros2 launch nav2_bringup bringup_launch.py use_sim_time:=true map:=/home/penguin/robot_ws/src/my_mapper_turtlebot_pkg/maps/stress_maze_map.yaml params_file:=/home/penguin/robot_ws/src/my_mapper_turtlebot_pkg/config/nav2_params_stress_maze.yaml autostart:=true use_composition:=False use_respawn:=False`
  3. `ros2 run my_mapper_turtlebot_pkg mapper_explorer`
  4. `rviz2 -d /home/penguin/robot_ws/src/my_mapper_turtlebot_pkg/rviz/explorer.rviz`

## 4. 의존성
- Nav2 bringup, SLAM, `/map`, `/odom`, `/navigate_to_pose`
- `stress_maze_explorer.launch.py`는 world/map 리소스를 `my_mapper_turtlebot_pkg` 내부에서 사용한다.

## 5. 주요 탐색 파라미터(현재값)
- Goal timeout: `45s`
- 정체 판정: `progress_stall_reset_sec = 18s`
- 정체 취소 후 블랙리스트 초기화 유예: `post_stall_blacklist_clear_grace_sec = 45s`
- 정체 취소 후 병목 재진입 억제:
  - `post_stall_hard_blacklist_boost_sec = 60s`
  - `post_stall_hard_blacklist_radius_boost = 0.50m`
- 최근 goal 재선정 억제:
  - `recent_goals_maxlen = 8`
  - `recent_goal_radius = 1.00m`
  - `recent_goal_penalty = 2.3`
- 장애물/안전거리 관련:
  - `max_obstacle_density = 0.24`
  - `min_clearance_radius_cells = 2`
  - `hard_blacklist_radius = 0.95m`
  - `hard_blacklist_ttl_sec = 120s`
  - `stalled_hotspot_trigger_count = 1`

## 6. Nav2 튜닝 포인트(현재값)
- `config/nav2_params_stress_maze.yaml` 기준
- Controller:
  - `controller_frequency = 15.0`
  - `failure_tolerance = 0.5`
  - `FollowPath.max_vel_theta = 0.8`
  - `FollowPath.BaseObstacle.scale = 0.15`
- Costmap:
  - `local_costmap width/height = 4`
  - `local/global inflation_radius = 0.65`
- Planner/Behavior:
  - `expected_planner_frequency = 10.0`
  - `behavior_server.max_rotational_vel = 0.8`
