# my_mapper_turtlebot_pkg

`my_gui_turtlebot_pkg`에서 mapper 관련 파일만 분리한 패키지입니다.

## Build
```bash
cd ~/robot_ws
colcon build --packages-select my_mapper_turtlebot_pkg
source /opt/ros/humble/setup.bash
source ~/robot_ws/install/setup.bash
```

## 실행 (시뮬)
- 탐색 노드만 실행:
  - `ros2 launch my_mapper_turtlebot_pkg mapper_explorer.launch.py use_sim_time:=true`
- stress maze 통합 실행:
  - `ros2 launch my_mapper_turtlebot_pkg stress_maze_explorer.launch.py`
- 분리 실행(권장 순서):
  1. `ros2 launch my_mapper_turtlebot_pkg stress_maze_world.launch.py use_sim_time:=true`
  2. `ros2 launch nav2_bringup bringup_launch.py use_sim_time:=true map:=/home/penguin/robot_ws/src/my_mapper_turtlebot_pkg/maps/stress_maze_map.yaml params_file:=/home/penguin/robot_ws/src/my_mapper_turtlebot_pkg/config/nav2_params_stress_maze.yaml autostart:=true use_composition:=False use_respawn:=False`
  3. `ros2 run my_mapper_turtlebot_pkg mapper_explorer --ros-args -p use_sim_time:=true`
  4. `rviz2 -d /home/penguin/robot_ws/src/my_mapper_turtlebot_pkg/rviz/explorer.rviz`

## 실행 (실기)
1. 로봇 bringup (로봇 PC)
   - `source /opt/ros/humble/setup.bash`
   - `export ROS_DOMAIN_ID=4`
   - `export TURTLEBOT3_MODEL=burger`
   - `export LDS_MODEL=LDS-01`
   - `ros2 launch turtlebot3_bringup robot.launch.py`
2. Nav2 + SLAM (원격 PC)
   - `source /opt/ros/humble/setup.bash`
   - `source ~/robot_ws/install/setup.bash`
   - `export ROS_DOMAIN_ID=4`
   - `ros2 launch nav2_bringup bringup_launch.py use_sim_time:=false slam:=True map:=/home/penguin/robot_ws/src/my_mapper_turtlebot_pkg/maps/stress_maze_map.yaml params_file:=/home/penguin/robot_ws/src/my_mapper_turtlebot_pkg/config/nav2_params_real.yaml autostart:=true use_composition:=False use_respawn:=False`
3. Explorer
   - `ros2 run my_mapper_turtlebot_pkg mapper_explorer --ros-args -p use_sim_time:=false`
4. RViz (선택)
   - `rviz2 -d /home/penguin/robot_ws/src/my_mapper_turtlebot_pkg/rviz/explorer.rviz`

## 현재 핵심 튜닝값
- goal timeout: `45s`
- 정체 판정: `28s`
- goal 재선정 쿨다운: `4s`
- 기본 goal 최소거리: `0.35m`
- rescue 최소거리(단계별): `0.15m`, `0.10m`
- obstacle density 상한: `0.22`
- hard blacklist: 반경 `0.60m`, TTL `45s`
- blacklists relax 시작: `12s`
- 초기 스캔 갱신: 시작 후 제자리 회전 `2s` (`/cmd_vel`)
- 실기 Nav2 goal tolerance: `xy_goal_tolerance = 0.10`

## Explorer 주요 파라미터
- `initial_spin_enabled` (기본 `true`)
- `initial_spin_duration_sec` (기본 `2.0`)
- `initial_spin_angular_vel` (기본 `0.60`)
- `goal_reselection_cooldown_sec` (기본 `4.0`)

## 운영 팁
- Nav2 로그에 `Managed nodes are active`가 뜬 뒤 `mapper_explorer`를 실행하세요.
- 실기에서 `planner loop missed ...`가 자주 뜨면 RViz를 끄거나 디스플레이를 최소화하세요.
