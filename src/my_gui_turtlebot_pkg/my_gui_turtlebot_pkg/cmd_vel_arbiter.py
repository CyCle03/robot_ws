import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_srvs.srv import SetBool
from std_srvs.srv import Trigger


class CmdVelArbiter(Node):
    def __init__(self):
        super().__init__('cmd_vel_arbiter')
        qos = QoSProfile(depth=10)

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', qos)

        self.manual_enabled = True
        self.patrol_enabled = True
        self.avoid_enabled = True
        self.emergency_stop = False
        self.source_timeout_sec = 0.5
        self.zero_twist = Twist()

        self.manual_msg = Twist()
        self.patrol_msg = Twist()
        self.avoid_msg = Twist()
        self.manual_stamp = None
        self.patrol_stamp = None
        self.avoid_stamp = None

        self.create_subscription(Twist, '/cmd_vel_manual', self.manual_callback, qos)
        self.create_subscription(Twist, '/cmd_vel_patrol', self.patrol_callback, qos)
        self.create_subscription(Twist, '/cmd_vel_avoid', self.avoid_callback, qos)

        self.create_service(SetBool, 'set_manual_enabled', self.set_manual_enabled_callback)
        self.create_service(SetBool, 'set_patrol_enabled', self.set_patrol_enabled_callback)
        self.create_service(SetBool, 'set_avoid_enabled', self.set_avoid_enabled_callback)
        self.create_service(SetBool, 'set_emergency_stop', self.set_emergency_stop_callback)
        self.create_service(Trigger, 'emergency_stop', self.emergency_stop_callback)

        self.timer = self.create_timer(0.05, self.publish_selected_cmd)

    def _now_sec(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _is_recent(self, stamp_sec):
        if stamp_sec is None:
            return False
        return (self._now_sec() - stamp_sec) <= self.source_timeout_sec

    def _is_non_zero(self, msg):
        return abs(msg.linear.x) > 1e-6 or abs(msg.angular.z) > 1e-6

    def manual_callback(self, msg):
        self.manual_msg = msg
        self.manual_stamp = self._now_sec()

    def patrol_callback(self, msg):
        self.patrol_msg = msg
        self.patrol_stamp = self._now_sec()

    def avoid_callback(self, msg):
        self.avoid_msg = msg
        self.avoid_stamp = self._now_sec()

    def set_manual_enabled_callback(self, request, response):
        self.manual_enabled = request.data
        response.success = True
        response.message = f'manual enabled={self.manual_enabled}'
        return response

    def set_patrol_enabled_callback(self, request, response):
        self.patrol_enabled = request.data
        response.success = True
        response.message = f'patrol enabled={self.patrol_enabled}'
        return response

    def set_avoid_enabled_callback(self, request, response):
        self.avoid_enabled = request.data
        response.success = True
        response.message = f'avoid enabled={self.avoid_enabled}'
        return response

    def set_emergency_stop_callback(self, request, response):
        self.emergency_stop = request.data
        if self.emergency_stop:
            self.cmd_vel_pub.publish(self.zero_twist)
        response.success = True
        response.message = f'emergency_stop={self.emergency_stop}'
        return response

    def emergency_stop_callback(self, request, response):
        del request
        self.emergency_stop = True
        self.cmd_vel_pub.publish(self.zero_twist)
        response.success = True
        response.message = 'emergency stop enabled'
        return response

    def publish_selected_cmd(self):
        if self.emergency_stop:
            self.cmd_vel_pub.publish(self.zero_twist)
            return

        if (
            self.avoid_enabled
            and self._is_recent(self.avoid_stamp)
            and self._is_non_zero(self.avoid_msg)
        ):
            self.cmd_vel_pub.publish(self.avoid_msg)
            return

        if (
            self.patrol_enabled
            and self._is_recent(self.patrol_stamp)
            and self._is_non_zero(self.patrol_msg)
        ):
            self.cmd_vel_pub.publish(self.patrol_msg)
            return

        if self.manual_enabled and self._is_recent(self.manual_stamp):
            self.cmd_vel_pub.publish(self.manual_msg)
            return

        self.cmd_vel_pub.publish(self.zero_twist)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelArbiter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
