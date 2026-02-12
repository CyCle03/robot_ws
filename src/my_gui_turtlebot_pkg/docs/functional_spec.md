# 기능명세서 (my_gui_turtlebot_pkg)

## 1. 목적
- GUI 수동 제어, Patrol Action 제어, 장애물 회피 제어를 통합하고 최종 `/cmd_vel` 출력을 단일 노드에서 중재한다.

## 2. 아키텍처 개요
- 수동 제어 노드: `turtlebot_move_con.py` + `move_turtlebot_pub.py`
- 액션 클라이언트: `patrol_action_client.py`
- 액션 서버: `patrol_action_server.py`
- 장애물 회피: `detect_obstacle.py`
- 속도 중재기: `cmd_vel_arbiter.py`
- 통합 실행 launch: `launch/gui_turtlebot_system.launch.py`
- Frontier 탐색 노드: `mapper_explorer.py`
- Frontier 단독 실행 launch: `launch/mapper_explorer.launch.py`
- RViz 설정 파일: `rviz/explorer.rviz`

## 3. 토픽 명세
- 입력 토픽(중재기 기준):
  - `/cmd_vel_manual` (`geometry_msgs/msg/Twist`): GUI 수동 속도
  - `/cmd_vel_patrol` (`geometry_msgs/msg/Twist`): Patrol 액션 속도
  - `/cmd_vel_avoid` (`geometry_msgs/msg/Twist`): 장애물 회피 속도
- 출력 토픽:
  - `/cmd_vel` (`geometry_msgs/msg/Twist`): 로봇 최종 구동 명령
- 기타:
  - `scan` (`sensor_msgs/msg/LaserScan`): 장애물 회피 입력
  - `odom` (`nav_msgs/msg/Odometry`): Patrol 회전 제어 입력

## 4. 중재(Arbiter) 규칙
- 파일: `my_gui_turtlebot_pkg/cmd_vel_arbiter.py`
- 우선순위:
  - `avoid > patrol > manual`
- 소스 타임아웃:
  - 기본 `1.0s` 내 최신 메시지만 유효
- 비상정지:
  - 활성화 시 모든 입력을 무시하고 `Twist(0,0)`만 출력

## 5. Service 명세
- `set_obstacle_avoid_enabled` (`std_srvs/srv/SetBool`)
  - 서버: `detect_obstacle.py`
  - 기능: 장애물 회피 ON/OFF
- `set_manual_enabled` (`std_srvs/srv/SetBool`)
  - 서버: `cmd_vel_arbiter.py`
  - 기능: 수동 입력 반영 ON/OFF
- `set_patrol_enabled` (`std_srvs/srv/SetBool`)
  - 서버: `cmd_vel_arbiter.py`
  - 기능: Patrol 입력 반영 ON/OFF
- `set_avoid_enabled` (`std_srvs/srv/SetBool`)
  - 서버: `cmd_vel_arbiter.py`
  - 기능: Avoid 입력 반영 ON/OFF
- `set_emergency_stop` (`std_srvs/srv/SetBool`)
  - 서버: `cmd_vel_arbiter.py`
  - 기능: 비상정지 상태 ON/OFF
- `emergency_stop` (`std_srvs/srv/Trigger`)
  - 서버: `cmd_vel_arbiter.py`
  - 기능: 비상정지 즉시 활성화

## 6. Action 명세
- 액션 타입: `turtlebot3_msgs/action/Patrol`
- 액션 이름: `turtlebot3`
- 클라이언트: `patrol_action_client.py`
  - goal 전송, feedback 수신, result 수신, cancel 요청
- 서버: `patrol_action_server.py`
  - goal 수락/실행, feedback 발행, result 반환
  - cancel 요청 수락 및 주행 루프 중 취소 처리 지원

## 7. GUI 기능
- 파일: `my_gui_turtlebot_pkg/turtlebot_move_con.py`
- 버튼 기능:
  - `Go/Back/Left/Right/Stop`: 수동 속도 제어
  - `patrol(sqr)/patrol(tri)`: Patrol goal 전송
  - `patrol stop`: Patrol cancel + 수동 stop 호출
- 수동 속도 명령은 `/cmd_vel_manual`로 주기 발행(10Hz)

## 8. 실행 방법
- 단일 launch 실행:
  - `ros2 launch my_gui_turtlebot_pkg gui_turtlebot_system.launch.py`
- launch 포함 노드:
  - `cmd_vel_arbiter`
  - `patrol_action_server`
  - `detect_obstacle`
  - `turtlebot_move_con`

## 9. 의존성
- `rclpy`
- `action_msgs`
- `geometry_msgs`
- `nav_msgs`
- `nav2_msgs`
- `sensor_msgs`
- `std_srvs`
- `turtlebot3_msgs`
- `launch`, `launch_ros`

## 10. Frontier 탐색(MapperExplorer) 명세
- 파일: `my_gui_turtlebot_pkg/mapper_explorer.py`
- 입력:
  - `/map` (`nav_msgs/msg/OccupancyGrid`, `TRANSIENT_LOCAL`)
  - `/odom` (`nav_msgs/msg/Odometry`)
- 출력:
  - `NavigateToPose` goal 전송(`/navigate_to_pose`)
- 상태 머신:
  - `IDLE -> SELECT_GOAL -> NAVIGATING -> EVALUATE -> (반복 또는 DONE)`
- 탐색 단계:
  - `RICH`: 장애물 밀도 높은 frontier 우선
  - `COVERAGE`: 남은 frontier 커버리지 탐색

## 11. Frontier 선택/예외 처리 정책
- Frontier 추출:
  - `free(0)` 셀 중 주변 8-neighbor에 `unknown(-1)`이 존재하는 셀을 후보로 추출
  - BFS 클러스터링 후 클러스터 중심점을 goal 후보로 사용
- 선택 필터:
  - 로봇과 너무 가까운 goal 제외: `distance < 0.35m`
  - 장애물 과밀 goal 제외: `obstacle_density > 0.45`
- 점수식:
  - `score = w_obs*obs + w_info*info - w_dist*distance - w_visit*visited_penalty`
- 타임아웃/재시도:
  - goal timeout: `120s`
  - timeout 취소 발생 시 해당 goal 즉시 blacklist
  - goal 재시도 제한: `max_goal_retries = 1`

## 12. 매핑 실행 시 주의사항
- SLAM은 하나만 실행해야 함 (`slam_toolbox`와 `cartographer` 동시 실행 금지)
- `mapper_explorer.launch.py`는 탐색 노드만 실행하므로 아래 스택이 별도로 선행되어야 함:
  - Gazebo/Robot bringup (`/odom`, TF)
  - SLAM (`/map`)
  - Nav2 (`/navigate_to_pose`)
