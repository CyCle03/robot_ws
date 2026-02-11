import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

class Detect_turtle(Node):
  def __init__(self):
    super().__init__('detect_turtle')
    self.qos_profile = QoSProfile(depth = 10)
    self.scan_sub = self.create_subscription(
      LaserScan,
      'scan',
      self.scan_callback,
      qos_profile=qos_profile_sensor_data)
    self.velocity = 0.0
    self.angular = 0.0
    self.scan_ranges = []

  def scan_callback(self, msg):
    self.scan_ranges = msg.ranges
    self.has_scan_received = True
    scan_range = len(self.scan_ranges) -1
    left_range = int(scan_range / 4)
    right_range = int(scan_range * 3 / 4)
    left_min = min(self.scan_ranges[0:left_range])
    right_min = min(self.scan_ranges[right_range:scan_range])
    self.get_logger().info(f'left_min:{left_min},right_min: {right_min}', throttle_duration_sec=2)
    self.get_logger().info(f'scanData: {self.scan_ranges[0]}', throttle_duration_sec=2)


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
