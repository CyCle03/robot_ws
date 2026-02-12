import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from turtlebot3_msgs.action import Patrol

class PatrolActionClient(Node):
    def __init__(self):
        super().__init__('patrol_action_client')
        print('TurtleBot3 Patrol Client')
        self._action_client = ActionClient(self, Patrol, 'turtlebot3')
        self._send_goal_future = None
        self._get_result_future = None
        self._goal_handle = None

    def send_goal(self, mode: int, travel_distance: float, patrol_count: int):
        goal_msg = Patrol.Goal()
        goal_msg.goal.x = float(mode)
        goal_msg.goal.y = float(travel_distance)
        goal_msg.goal.z = float(patrol_count)

        if not self._action_client.wait_for_server(timeout_sec=0.5):
            self.get_logger().warning('Action server not available: turtlebot3')
            return

        self._send_goal_future = \
            self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self._goal_handle = goal_handle
        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Feedback: {0}'.format(feedback.state))

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.result))
        self._goal_handle = None

    def cancel_goal(self):
        if self._goal_handle is None:
            self.get_logger().info('No active goal to cancel.')
            return
        cancel_future = self._goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(self.cancel_done_callback)

    def cancel_done_callback(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info('Goal cancel requested.')
        else:
            self.get_logger().info('Goal cancel rejected.')

def main(args=None):
    rclpy.init(args=args)
    patrol_action_client = PatrolActionClient()
    rclpy.spin(patrol_action_client)


if __name__ == '__main__':
    main()
