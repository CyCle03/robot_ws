# my_mapper_turtlebot_pkg

`my_gui_turtlebot_pkg`에서 mapper 관련 파일만 분리한 패키지입니다.

## 실행
- `ros2 launch my_mapper_turtlebot_pkg mapper_explorer.launch.py`
- `ros2 launch my_mapper_turtlebot_pkg stress_maze_explorer.launch.py`

## 기본 튜닝값
- 정체 판정 `30s`, goal timeout `60s`, 최근 goal/병목 재진입 억제 로직 포함
