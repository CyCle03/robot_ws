import math
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_srvs.srv import SetBool

class DetectObstacle(Node):
    def __init__(self):
        super().__init__('detect_obstacle')
        self.enabled = True
        self.stop_distance = 0.45
        self.clear_distance = 0.58
        self.turn_speed = 0.7
        self.backup_speed = -0.08
        self.backup_duration_sec = 0.25
        self.turn_duration_sec = 0.5
        self.scan_ranges = []
        self.has_scan_received = False
        self.recovery_phase = 'idle'
        self.phase_end_sec = 0.0
        self.recovery_turn_direction = 1.0
        self.qos_profile = QoSProfile(depth=10)

        self.scan_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile=qos_profile_sensor_data
        )
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel_avoid', self.qos_profile)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.enable_srv = self.create_service(
            SetBool,
            'set_obstacle_avoid_enabled',
            self.set_enabled_callback
        )

    def set_enabled_callback(self, request, response):
        self.enabled = request.data
        response.success = True
        response.message = f'obstacle avoid enabled={self.enabled}'
        return response

    def scan_callback(self, msg):
        self.scan_ranges = msg.ranges
        self.has_scan_received = True

    def _get_valid_min(self, values):
        valid_values = [v for v in values if math.isfinite(v) and v > 0.0]
        if not valid_values:
            return float('inf')
        return min(valid_values)

    def _now_sec(self):
        return self.get_clock().now().nanoseconds / 1e9

    def timer_callback(self):
        twist = Twist()
        if not self.enabled or not self.has_scan_received:
            self.recovery_phase = 'idle'
            self.cmd_vel_pub.publish(twist)
            return

        scan_len = len(self.scan_ranges)
        if scan_len == 0:
            self.recovery_phase = 'idle'
            self.cmd_vel_pub.publish(twist)
            return

        now = self._now_sec()
        left_range = int(scan_len / 4)
        right_range = int(scan_len * 3 / 4)
        front_left_min = self._get_valid_min(self.scan_ranges[0:left_range])
        front_right_min = self._get_valid_min(self.scan_ranges[right_range:scan_len])
        obstacle_distance = min(front_left_min, front_right_min)

        # Recovery phase 1: brief backup to create clearance from tight obstacle gaps.
        if self.recovery_phase == 'backup':
            if now < self.phase_end_sec:
                twist.linear.x = self.backup_speed
                self.cmd_vel_pub.publish(twist)
                return
            self.recovery_phase = 'turn'
            self.phase_end_sec = now + self.turn_duration_sec

        # Recovery phase 2: rotate toward the wider side.
        if self.recovery_phase == 'turn':
            if obstacle_distance > self.clear_distance:
                self.recovery_phase = 'idle'
                self.cmd_vel_pub.publish(twist)
                return
            if now < self.phase_end_sec:
                twist.angular.z = self.recovery_turn_direction * self.turn_speed
                self.cmd_vel_pub.publish(twist)
                return
            self.recovery_phase = 'idle'
            self.cmd_vel_pub.publish(twist)
            return

        if obstacle_distance < self.stop_distance:
            # front_left_min > front_right_min 이면 왼쪽 공간이 더 넓으므로 좌회전 방향 선택
            self.recovery_turn_direction = 1.0 if front_left_min > front_right_min else -1.0
            self.recovery_phase = 'backup'
            self.phase_end_sec = now + self.backup_duration_sec
            twist.linear.x = self.backup_speed

        self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = DetectObstacle()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
