import math
import time

from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import ActionServer
from rclpy.action import CancelResponse
from rclpy.action import GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import QoSProfile

from turtlebot3_msgs.action import Patrol


class PatrolActionServer(Node):

    def __init__(self):
        super().__init__('patrol_action_server')

        print('Patrol Action Server')
        print('----------------------------------------------')

        self._action_server = ActionServer(
            self,
            Patrol,
            'turtlebot3',
            self.execute_callback,
            callback_group=ReentrantCallbackGroup(),
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback)

        self.goal_msg = Patrol.Goal()
        self.twist = Twist()
        self.odom = Odometry()
        self.position = Point()
        self.rotation = 0.0

        self.linear_x = 1.0
        self.angular_z = 4.0

        qos = QoSProfile(depth=10)

        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel_patrol', qos)

        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.odom_callback, qos
        )

    def init_twist(self):
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        self.cmd_vel_pub.publish(self.twist)

    def odom_callback(self, msg):
        self.odom = msg

    def get_yaw(self):
        q = self.odom.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny, cosy)

    def go_front(self, goal_handle, position, length):
        step_dt = 0.1
        while True:
            if goal_handle.is_cancel_requested:
                self.init_twist()
                return False
            if position >= length:
                break
            self.twist.linear.x = self.linear_x
            self.twist.angular.z = 0.0
            self.cmd_vel_pub.publish(self.twist)

            time.sleep(step_dt)
            position += abs(self.linear_x) * step_dt
        self.init_twist()
        return True

    def turn(self, goal_handle, target_angle):
        initial_yaw = self.get_yaw()
        target_yaw = initial_yaw + (target_angle * math.pi / 180.0)

        while True:
            if goal_handle.is_cancel_requested:
                self.init_twist()
                return False
            rclpy.spin_once(self, timeout_sec=0.1)

            current_yaw = self.get_yaw()
            yaw_diff = abs(
                math.atan2(
                    math.sin(target_yaw - current_yaw),
                    math.cos(target_yaw - current_yaw)
                )
            )

            if yaw_diff < 0.01:
                break

            self.twist.linear.x = 0.0
            self.twist.angular.z = self.angular_z
            self.cmd_vel_pub.publish(self.twist)

        self.init_twist()
        return True

    def goal_callback(self, goal_request):
        self.goal_msg = goal_request

        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel request.')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        feedback_msg = Patrol.Feedback()
        result = Patrol.Result()

        length = self.goal_msg.goal.y
        iteration = int(self.goal_msg.goal.z)

        if self.goal_msg.goal.x == 1:
            for _ in range(iteration):
                if not self.square(feedback_msg, goal_handle, length):
                    goal_handle.canceled()
                    result.result = 'patrol canceled'
                    self.get_logger().info('Patrol canceled.')
                    return result
            feedback_msg.state = 'square patrol complete!!'
        elif self.goal_msg.goal.x == 2:
            for _ in range(iteration):
                if not self.triangle(feedback_msg, goal_handle, length):
                    goal_handle.canceled()
                    result.result = 'patrol canceled'
                    self.get_logger().info('Patrol canceled.')
                    return result
            feedback_msg.state = 'triangle patrol complete!!'
        else:
            goal_handle.abort()
            result.result = 'invalid patrol mode'
            self.get_logger().warning('Invalid patrol mode.')
            return result

        goal_handle.succeed()
        result.result = feedback_msg.state

        self.init_twist()
        self.get_logger().info('Patrol complete.')

        return result

    def square(self, feedback_msg, goal_handle, length):
        self.linear_x = 0.2
        self.angular_z = 13 * (90.0 / 180.0) * math.pi / 100.0

        for i in range(4):
            self.position.x = 0.0
            self.angle = 0.0

            if not self.go_front(goal_handle, self.position.x, length):
                return False
            if not self.turn(goal_handle, 90.0):
                return False

            feedback_msg.state = 'line ' + str(i + 1)
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(0.1)

        self.init_twist()
        return True

    def triangle(self, feedback_msg, goal_handle, length):
        self.linear_x = 0.2
        self.angular_z = 8 * (120.0 / 180.0) * math.pi / 100.0

        for i in range(3):
            self.position.x = 0.0
            self.angle = 0.0

            if not self.go_front(goal_handle, self.position.x, length):
                return False
            if not self.turn(goal_handle, 120.0):
                return False

            feedback_msg.state = 'line ' + str(i + 1)
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1)

        self.init_twist()
        return True


def main(args=None):
    rclpy.init(args=args)

    patrol_action_server = PatrolActionServer()

    rclpy.spin(patrol_action_server)


if __name__ == '__main__':
    main()
