import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
import math

class Detect_turtle(Node):
    def __init__(self):
        super().__init__('detect_turtle')
        self.stop_distance = 0.5
        self.forward_speed = 0.12
        self.turn_speed = 0.7
        self.turn_duration = 1.5
        self.scan_ranges = []
        self.has_scan_received = False
        self.is_turning = False
        self.turn_direction = 0.0
        self.turn_end_time_ns = 0
        self.qos_profile = QoSProfile(depth=10)
        self.scan_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile=qos_profile_sensor_data)

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', self.qos_profile)

    def scan_callback(self, msg):
        self.scan_ranges = msg.ranges
        self.has_scan_received = True

    def _get_valid_min(self, values):
        valid_values = [v for v in values if math.isfinite(v) and v > 0.0]
        if not valid_values:
            return float('inf')
        return min(valid_values)

    def timer_callback(self):
        twist = Twist()
        if not self.has_scan_received:
            return

        scan_len = len(self.scan_ranges)
        left_range = int(scan_len / 4)
        right_range = int(scan_len * 3 / 4)
        left_min = self._get_valid_min(self.scan_ranges[0:left_range])
        right_min = self._get_valid_min(self.scan_ranges[right_range:scan_len])
        obstacle_distance = min(left_min, right_min)

        now_ns = self.get_clock().now().nanoseconds
        if self.is_turning:
            if now_ns < self.turn_end_time_ns:
                twist.linear.x = 0.0
                twist.angular.z = self.turn_direction * self.turn_speed
            else:
                self.is_turning = False
                twist.linear.x = self.forward_speed
                twist.angular.z = 0.0
                self.get_logger().info('Turn complete. Moving forward.')
        else:
            if obstacle_distance < self.stop_distance:
                # left_min > right_min 이면 왼쪽 공간이 더 넓으므로 좌회전
                self.turn_direction = 1.0 if left_min > right_min else -1.0
                self.is_turning = True
                self.turn_end_time_ns = now_ns + int(self.turn_duration * 1e9)
                twist.linear.x = 0.0
                twist.angular.z = self.turn_direction * self.turn_speed
                turn_text = 'left' if self.turn_direction > 0 else 'right'
                self.get_logger().info(
                    f'Obstacle detected! left_min={left_min:.2f}, right_min={right_min:.2f} -> turn {turn_text}.')
            else:
                twist.linear.x = self.forward_speed
                twist.angular.z = 0.0

        self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = Detect_turtle()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt!!!!')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
