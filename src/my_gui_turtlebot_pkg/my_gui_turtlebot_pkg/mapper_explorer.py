from enum import Enum

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from .mapper_frontier_utils import extract_frontiers
from .mapper_frontier_utils import select_goal


class ExplorerState(Enum):
    IDLE = 0
    SELECT_GOAL = 1
    NAVIGATING = 2
    EVALUATE = 3
    DONE = 4


class Phase(Enum):
    RICH = 0
    COVERAGE = 1


class MapperExplorer(Node):
    def __init__(self):
        super().__init__('mapper_explorer')
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.map_msg = None
        self.robot_x = None
        self.robot_y = None
        self._map_rx_logged = False
        self._odom_rx_logged = False
        self.state = ExplorerState.IDLE
        self.phase = Phase.RICH
        self.goal_active = False
        self.goal_handle = None
        self.goal_sent_time_sec = None
        self.current_goal = None
        self.blacklist = set()
        self.hard_blacklist = set()
        self.hard_blacklist_time = {}
        self.visited_count = {}
        self.goal_fail_count = {}
        self.last_result = None
        self.last_status = None
        self.last_goal_selected_time_sec = None

        self.rich_min_density = 0.0
        self.goal_timeout_sec = 120.0
        self.max_goal_retries = 0
        self.frontier_min_cluster_size = 3
        self.w_dist = 0.7
        self.w_obs = 0.6
        self.w_info = 1.8
        self.w_visit = 0.8
        self.min_goal_distance = 0.45
        self.max_obstacle_density = 0.25
        self.blacklist_radius = 0.80
        self.hard_blacklist_radius = 1.00
        self.map_margin_cells = 2
        self.min_clearance_radius_cells = 2
        self.hard_blacklist_ttl_sec = 20.0
        self.no_goal_relax_after_sec = 12.0

        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, map_qos)
        self.create_subscription(Odometry, '/odom', self.odom_callback, qos_profile_sensor_data)
        self.timer = self.create_timer(1.0, self.step)

    def map_callback(self, msg):
        self.map_msg = msg
        if not self._map_rx_logged:
            self.get_logger().info('Received /map')
            self._map_rx_logged = True

    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        if not self._odom_rx_logged:
            self.get_logger().info('Received /odom')
            self._odom_rx_logged = True

    def step(self):
        if self.map_msg is None or self.robot_x is None:
            return
        self._prune_hard_blacklist()

        if self.state == ExplorerState.IDLE:
            self.state = ExplorerState.SELECT_GOAL
            return

        if self.state == ExplorerState.SELECT_GOAL:
            frontiers = self.extract_frontiers(self.map_msg)
            self.get_logger().info(f'Frontier clusters: {len(frontiers)}')
            if not frontiers:
                if self.phase == Phase.RICH:
                    self.phase = Phase.COVERAGE
                    self.get_logger().info('Switch phase: RICH -> COVERAGE')
                    return
                self.state = ExplorerState.DONE
                self.get_logger().info('No frontiers left. Exploration done.')
                return

            goal = self.select_goal_with_fallback(frontiers)
            if goal is None:
                if self.phase == Phase.RICH:
                    self.phase = Phase.COVERAGE
                    self.get_logger().info('No rich frontier. Switch to COVERAGE.')
                else:
                    self.state = ExplorerState.DONE
                return

            self.send_nav_goal(goal)
            self.current_goal = goal
            self.last_goal_selected_time_sec = self._now_sec()
            self.state = ExplorerState.NAVIGATING
            return

        if self.state == ExplorerState.NAVIGATING:
            if not self.goal_active:
                self.state = ExplorerState.EVALUATE
                return
            if self.goal_sent_time_sec is not None:
                elapsed = self._now_sec() - self.goal_sent_time_sec
                if elapsed > self.goal_timeout_sec:
                    if self.goal_handle is None:
                        self.get_logger().warning('Goal timeout before acceptance.')
                        if self.current_goal is not None:
                            key = (round(self.current_goal[0], 2), round(self.current_goal[1], 2))
                            self.blacklist.add(key)
                            self._add_hard_blacklist(key)
                        self.goal_active = False
                        self.last_status = GoalStatus.STATUS_ABORTED
                        self.state = ExplorerState.EVALUATE
                    else:
                        self.get_logger().warning('Goal timeout. Cancel navigation goal.')
                        self.cancel_active_goal(reason='timeout')
            return

        if self.state == ExplorerState.EVALUATE:
            if self.current_goal is not None:
                key = (round(self.current_goal[0], 2), round(self.current_goal[1], 2))
                if self.last_status == GoalStatus.STATUS_SUCCEEDED:
                    self.visited_count[key] = self.visited_count.get(key, 0) + 1
                    self.goal_fail_count[key] = 0
                else:
                    fail = self.goal_fail_count.get(key, 0) + 1
                    self.goal_fail_count[key] = fail
                    if fail > self.max_goal_retries:
                        self.blacklist.add(key)
                        self._add_hard_blacklist(key)

            self.current_goal = None
            self.last_result = None
            self.last_status = None
            self.state = ExplorerState.SELECT_GOAL

    def _now_sec(self):
        return self.get_clock().now().nanoseconds / 1e9

    def extract_frontiers(self, map_msg):
        return extract_frontiers(map_msg, self.frontier_min_cluster_size)

    def select_goal(self, frontiers):
        return self._select_goal(frontiers)

    def _active_hard_blacklist(self):
        now = self._now_sec()
        return {
            key for key, ts in self.hard_blacklist_time.items()
            if (now - ts) <= self.hard_blacklist_ttl_sec
        }

    def _prune_hard_blacklist(self):
        active = self._active_hard_blacklist()
        self.hard_blacklist = active
        self.hard_blacklist_time = {k: self.hard_blacklist_time[k] for k in active}

    def _add_hard_blacklist(self, key):
        self.hard_blacklist.add(key)
        self.hard_blacklist_time[key] = self._now_sec()

    def _select_goal(
        self,
        frontiers,
        *,
        max_obstacle_density=None,
        min_clearance_radius_cells=None,
        hard_blacklist_radius=None,
    ):
        w_obs = self.w_obs if self.phase == Phase.RICH else self.w_obs * 0.4
        return select_goal(
            frontiers=frontiers,
            map_msg=self.map_msg,
            robot_x=self.robot_x,
            robot_y=self.robot_y,
            blacklist=self.blacklist,
            hard_blacklist=self._active_hard_blacklist(),
            visited_count=self.visited_count,
            phase=self.phase.name,
            rich_min_density=self.rich_min_density,
            w_dist=self.w_dist,
            w_obs=w_obs,
            w_info=self.w_info,
            w_visit=self.w_visit,
            min_goal_distance=self.min_goal_distance,
            max_obstacle_density=max_obstacle_density
            if max_obstacle_density is not None else self.max_obstacle_density,
            blacklist_radius=self.blacklist_radius,
            hard_blacklist_radius=hard_blacklist_radius
            if hard_blacklist_radius is not None else self.hard_blacklist_radius,
            map_margin_cells=self.map_margin_cells,
            min_clearance_radius_cells=min_clearance_radius_cells
            if min_clearance_radius_cells is not None else self.min_clearance_radius_cells,
        )

    def select_goal_with_fallback(self, frontiers):
        attempts = [
            {},
            {'min_clearance_radius_cells': 1},
            {'min_clearance_radius_cells': 1, 'max_obstacle_density': 0.35},
            {
                'min_clearance_radius_cells': 1,
                'max_obstacle_density': 0.35,
                'hard_blacklist_radius': 0.50,
            },
        ]
        for params in attempts:
            goal = self._select_goal(frontiers, **params)
            if goal is not None:
                return goal

        if self.last_goal_selected_time_sec is not None:
            if (self._now_sec() - self.last_goal_selected_time_sec) > self.no_goal_relax_after_sec:
                self.get_logger().warning(
                    'No selectable goal for a while. Expire hard blacklist entries.'
                )
                self.hard_blacklist.clear()
                self.hard_blacklist_time.clear()
                return self._select_goal(frontiers, max_obstacle_density=0.35, min_clearance_radius_cells=1)
        return None

    def send_nav_goal(self, goal_xy):
        if not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warning('NavigateToPose server unavailable.')
            self.goal_active = False
            self.last_status = GoalStatus.STATUS_ABORTED
            self.state = ExplorerState.EVALUATE
            return

        gx, gy = goal_xy
        goal = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = gx
        pose.pose.position.y = gy
        pose.pose.orientation.w = 1.0
        goal.pose = pose

        self.goal_active = True
        self.goal_sent_time_sec = self._now_sec()
        self.get_logger().info(f'Send goal: x={gx:.2f}, y={gy:.2f}, phase={self.phase.name}')
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.get_logger().error(f'Goal response failed: {exc}')
            self.goal_active = False
            self.goal_handle = None
            self.last_status = GoalStatus.STATUS_ABORTED
            self.state = ExplorerState.EVALUATE
            return

        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected.')
            if self.current_goal is not None:
                key = (round(self.current_goal[0], 2), round(self.current_goal[1], 2))
                self.blacklist.add(key)
                self._add_hard_blacklist(key)
            self.goal_active = False
            self.goal_handle = None
            self.last_status = GoalStatus.STATUS_ABORTED
            self.state = ExplorerState.EVALUATE
            return

        self.goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        try:
            wrapped = future.result()
        except Exception as exc:
            self.get_logger().error(f'Result callback failed: {exc}')
            self.last_result = None
            self.last_status = GoalStatus.STATUS_ABORTED
            self.goal_active = False
            self.goal_handle = None
            self.goal_sent_time_sec = None
            self.state = ExplorerState.EVALUATE
            return

        self.last_result = wrapped.result
        self.last_status = wrapped.status
        self.goal_active = False
        self.goal_handle = None
        self.goal_sent_time_sec = None
        self.state = ExplorerState.EVALUATE

    def cancel_active_goal(self, reason='manual'):
        if self.goal_handle is None:
            return
        cancel_future = self.goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(
            lambda future: self.cancel_done_callback(future, reason)
        )

    def cancel_done_callback(self, future, reason):
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().error(f'Cancel callback failed ({reason}): {exc}')
            response = None
        if response is not None and len(response.goals_canceling) > 0:
            self.get_logger().info(f'Goal canceled ({reason}).')
            self.last_status = GoalStatus.STATUS_CANCELED
            if reason == 'timeout' and self.current_goal is not None:
                key = (round(self.current_goal[0], 2), round(self.current_goal[1], 2))
                self.blacklist.add(key)
                self._add_hard_blacklist(key)
        else:
            self.get_logger().warning(f'Goal cancel rejected ({reason}).')
            self.last_status = GoalStatus.STATUS_ABORTED
        self.goal_active = False
        self.goal_handle = None
        self.goal_sent_time_sec = None
        self.state = ExplorerState.EVALUATE


def main(args=None):
    rclpy.init(args=args)
    node = MapperExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
