import rclpy as rp
from rclpy.action import ActionServer
from my_first_package_msgs.action import DistTurtle
from rclpy.node import Node
import time

class DistTurtleServer(Node):

  def __init__(self):
    super().__init__('dist_turtle_action_server')
    self.action_server = ActionServer(self, DistTurtle, 'dist_turtle', self.excute_callback)

  def excute_callback(self, goal_handle):
    goal_handle.succeed()
    result = DistTurtle.Result()
    feedback_msg = DistTurtle.Feedback()
    for n in range(1,10):
      feedback_msg.remained_dist=float(n)
      goal_handle.publish_feedback(feedback_msg)
      time.sleep(0.5)
    return result


def main(args=None):
  rp.init(args=args)
  dist_turtle_action_server = DistTurtleServer()
  rp.spin(dist_turtle_action_server)

if __name__ == '__main__':
  main()
