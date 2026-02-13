# 기능명세서 (my_mapper_turtlebot_pkg)

## 1. 목적
- Frontier 탐색 관련 기능(`mapper_explorer`, `mapper_frontier_utils`)을 GUI/Patrol/Obstacle Avoid 패키지에서 분리해 독립 배포한다.

## 2. 포함 파일
- 노드: `my_mapper_turtlebot_pkg/mapper_explorer.py`
- 유틸: `my_mapper_turtlebot_pkg/mapper_frontier_utils.py`
- launch: `launch/mapper_explorer.launch.py`, `launch/stress_maze_explorer.launch.py`
- RViz 설정: `rviz/explorer.rviz`

## 3. 실행
- 탐색 노드만 실행:
  - `ros2 launch my_mapper_turtlebot_pkg mapper_explorer.launch.py`
- stress maze 통합 실행:
  - `ros2 launch my_mapper_turtlebot_pkg stress_maze_explorer.launch.py`

## 4. 의존성
- Nav2 bringup, SLAM, `/map`, `/odom`, `/navigate_to_pose`
- `stress_maze_explorer.launch.py`는 world/map 리소스를 `my_mapper_turtlebot_pkg` 내부에서 사용한다.

## 5. 주요 탐색 파라미터(현재값)
- Goal timeout: `60s`
- 정체 판정: `progress_stall_reset_sec = 30s`
- 정체 취소 후 블랙리스트 초기화 유예: `post_stall_blacklist_clear_grace_sec = 35s`
- 정체 취소 후 병목 재진입 억제:
  - `post_stall_hard_blacklist_boost_sec = 45s`
  - `post_stall_hard_blacklist_radius_boost = 0.35m`
- 최근 goal 재선정 억제:
  - `recent_goals_maxlen = 8`
  - `recent_goal_radius = 0.75m`
  - `recent_goal_penalty = 1.6`
