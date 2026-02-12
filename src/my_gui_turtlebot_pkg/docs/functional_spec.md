# 기능명세서 (TurtleBot GUI 이동 제어)

## 1. 목적
- GUI 버튼으로 TurtleBot의 선속도/각속도를 제어하고 `cmd_vel`을 주기적으로 발행한다.

## 2. 대상 파일
- `src/my_gui_turtlebot_pkg/my_gui_turtlebot_pkg/turtlebot_move_con.py`

## 3. 주요 기능
- `Go` 버튼: 선속도 증가 (`+0.2`, 범위 `-0.6 ~ 0.6`)
- `Back` 버튼: 선속도 감소 (`-0.2`, 범위 `-0.6 ~ 0.6`)
- `Left` 버튼: 각속도 증가 (`+0.1`, 범위 `-1.0 ~ 1.0`)
- `Right` 버튼: 각속도 감소 (`-0.1`, 범위 `-1.0 ~ 1.0`)
- `Stop` 버튼:
  - 제어 비활성화 (`enabled=False`)
  - 선속도/각속도 즉시 `0.0`으로 초기화
  - 정지 명령(`Twist(0,0)`) 1회 즉시 발행
- 주기 발행:
  - 타이머 주기 `0.1s`(10Hz)
  - `enabled=True`일 때만 현재 속도값 발행
  - `enabled=False`일 때 주기 발행 중단
- 로그 표시:
  - 발행값 `(linear.x, angular.z)`를 UI 리스트에 추가
  - 자동 스크롤

## 4. 입출력 명세
- 입력:
  - GUI 버튼 이벤트 (`btn_go`, `btn_back`, `btn_left`, `btn_right`, `btn_stop`)
- 출력:
  - ROS2 토픽 `cmd_vel` (`geometry_msgs/msg/Twist`)
  - UI 로그 리스트(`listWidget`)
  - ROS logger info

## 5. 상태 변수
- `velocity` (float): 선속도
- `angular` (float): 각속도
- `enabled` (bool): 주기 발행 활성/비활성
- `angular_step` (float): 각속도 증감폭 (`0.1`)
- `angular_limit` (float): 각속도 제한값 (`1.0`)

## 6. 제약/예외
- 모든 속도값은 `clamp()`로 제한 범위 내 유지
- 창 종료 시 executor/thread를 정상 종료하여 리소스 정리
