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
