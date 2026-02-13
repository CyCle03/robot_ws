# 기능명세서 (my_gui_turtlebot_pkg)

## 1. 목적
- GUI 수동 제어, Patrol Action 제어, 장애물 회피 제어를 통합하고 최종 `/cmd_vel` 출력을 단일 노드에서 중재한다.
- Mapper/Frontier 탐색 기능은 `my_mapper_turtlebot_pkg`로 분리했다.

## 2. 아키텍처 개요
- 수동 제어 노드: `turtlebot_move_con.py` + `move_turtlebot_pub.py`
- 액션 클라이언트: `patrol_action_client.py`
- 액션 서버: `patrol_action_server.py`
- 장애물 회피: `detect_obstacle.py`
- 속도 중재기: `cmd_vel_arbiter.py`
- 통합 실행 launch: `launch/gui_turtlebot_system.launch.py`

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
- `sensor_msgs`
- `std_srvs`
- `turtlebot3_msgs`
- `launch`, `launch_ros`

## 10. 분리된 Mapper 기능
- Mapper/Frontier 관련 실행은 `my_mapper_turtlebot_pkg`에서 제공한다.
- 예시:
  - `ros2 launch my_mapper_turtlebot_pkg mapper_explorer.launch.py`
  - `ros2 launch my_mapper_turtlebot_pkg stress_maze_explorer.launch.py`
