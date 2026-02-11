import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

class Detect_turtle(Node):
    def __init__(self):
        super().__init__('detect_turtle')
        self.stop_distance = 0.5
        self.scan_ranges = []
        self.has_scan_received = False
        self.qos_profile = QoSProfile(depth = 10)
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

    def timer_callback(self):
        twist = Twist()
        twist.linear.x = 0.1
        if not self.has_scan_received:
            return
        left_range = int(len(self.scan_ranges) / 4)
        right_range = int(len(self.scan_ranges) * 3 / 4)
        left_min = min(self.scan_ranges[0:left_range])
        right_min = min(self.scan_ranges[right_range:360])
        obstacle_distance = min(left_min, right_min)
        if obstacle_distance < self.stop_distance:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.get_logger().info('Obstacle detected! Stopping.', throttle_duration_sec=2)

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
