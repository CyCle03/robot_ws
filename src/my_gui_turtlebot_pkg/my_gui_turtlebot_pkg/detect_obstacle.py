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
        self.stop_distance = 0.5
        self.turn_speed = 0.7
        self.scan_ranges = []
        self.has_scan_received = False
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

    def timer_callback(self):
        twist = Twist()
        if not self.enabled or not self.has_scan_received:
            self.cmd_vel_pub.publish(twist)
            return

        scan_len = len(self.scan_ranges)
        if scan_len == 0:
            self.cmd_vel_pub.publish(twist)
            return

        left_range = int(scan_len / 4)
        right_range = int(scan_len * 3 / 4)
        left_min = self._get_valid_min(self.scan_ranges[0:left_range])
        right_min = self._get_valid_min(self.scan_ranges[right_range:scan_len])
        obstacle_distance = min(left_min, right_min)

        if obstacle_distance < self.stop_distance:
            # left_min > right_min 이면 왼쪽 공간이 더 넓으므로 좌회전
            turn_direction = 1.0 if left_min > right_min else -1.0
            twist.angular.z = turn_direction * self.turn_speed

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
