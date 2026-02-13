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
        self.goal_start_pose = None
        self.current_goal = None
        self.blacklist = set()
        self.blacklist_time = {}
        self.hard_blacklist = set()
        self.hard_blacklist_time = {}
        self.visited_count = {}
        self.goal_fail_count = {}
        self.last_result = None
        self.last_status = None
        self.last_goal_selected_time_sec = None

        self.rich_min_density = 0.0
        self.goal_timeout_sec = 120.0
        self.max_goal_retries = 1
        self.frontier_min_cluster_size = 3
        self.w_dist = 1.0
        self.w_obs = 0.6
        self.w_info = 1.8
        self.w_visit = 0.8
        self.min_goal_distance = 0.75
        self.distance_reward_cap_m = 2.5
        self.max_obstacle_density = 0.25
        self.blacklist_radius = 0.60
        self.hard_blacklist_radius = 0.60
        self.map_margin_cells = 2
        self.min_clearance_radius_cells = 2
        self.hard_blacklist_ttl_sec = 20.0
        self.blacklist_ttl_sec = 30.0
        self.no_goal_relax_after_sec = 12.0
        self.obstacle_density_radius_cells = 4
        self.false_success_min_planned_m = 0.35
        self.false_success_min_moved_m = 0.20
        self.goal_reached_tolerance_m = 0.20
        self.progress_min_distance_m = 0.20
        self.progress_stall_reset_sec = 20.0
        self.last_progress_pose = None
        self.last_progress_time_sec = None
        self.blacklist_clear_cooldown_sec = 8.0
        self.last_blacklist_clear_time_sec = None

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
        self._prune_blacklist()
        self._prune_hard_blacklist()
        self._update_progress_and_recover()

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
                            self._add_blacklist(key)
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
                        self._add_blacklist(key)
                    if fail > (self.max_goal_retries + 1):
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

    def _active_blacklist(self):
        now = self._now_sec()
        return {
            key for key, ts in self.blacklist_time.items()
            if (now - ts) <= self.blacklist_ttl_sec
        }

    def _prune_blacklist(self):
        active = self._active_blacklist()
        self.blacklist = active
        self.blacklist_time = {k: self.blacklist_time[k] for k in active}

    def _prune_hard_blacklist(self):
        active = self._active_hard_blacklist()
        self.hard_blacklist = active
        self.hard_blacklist_time = {k: self.hard_blacklist_time[k] for k in active}

    def _add_blacklist(self, key):
        self.blacklist.add(key)
        self.blacklist_time[key] = self._now_sec()

    def _add_hard_blacklist(self, key):
        self.hard_blacklist.add(key)
        self.hard_blacklist_time[key] = self._now_sec()

    def _clear_all_blacklists(self, reason):
        now = self._now_sec()
        if self.last_blacklist_clear_time_sec is not None:
            if (now - self.last_blacklist_clear_time_sec) < self.blacklist_clear_cooldown_sec:
                return False
        self.blacklist.clear()
        self.blacklist_time.clear()
        self.hard_blacklist.clear()
        self.hard_blacklist_time.clear()
        self.last_blacklist_clear_time_sec = now
        self.get_logger().warning(f'Clear blacklists: {reason}')
        return True

    def _update_progress_and_recover(self):
        now = self._now_sec()
        if self.last_progress_pose is None:
            self.last_progress_pose = (self.robot_x, self.robot_y)
            self.last_progress_time_sec = now
            return
        px, py = self.last_progress_pose
        moved = ((self.robot_x - px) ** 2 + (self.robot_y - py) ** 2) ** 0.5
        if moved >= self.progress_min_distance_m:
            self.last_progress_pose = (self.robot_x, self.robot_y)
            self.last_progress_time_sec = now
            return
        if self.last_progress_time_sec is not None:
            stalled = now - self.last_progress_time_sec
            if stalled >= self.progress_stall_reset_sec:
                cleared = self._clear_all_blacklists(
                    f'no meaningful motion for {stalled:.1f}s'
                )
                if cleared:
                    self.last_progress_pose = (self.robot_x, self.robot_y)
                    self.last_progress_time_sec = now

    def _select_goal(
        self,
        frontiers,
        *,
        max_obstacle_density=None,
        min_clearance_radius_cells=None,
        hard_blacklist_radius=None,
        blacklist_radius=None,
        min_goal_distance=None,
        rejection_stats=None,
    ):
        w_obs = self.w_obs if self.phase == Phase.RICH else self.w_obs * 0.4
        return select_goal(
            frontiers=frontiers,
            map_msg=self.map_msg,
            robot_x=self.robot_x,
            robot_y=self.robot_y,
            blacklist=self._active_blacklist(),
            hard_blacklist=self._active_hard_blacklist(),
            visited_count=self.visited_count,
            phase=self.phase.name,
            rich_min_density=self.rich_min_density,
            w_dist=self.w_dist,
            w_obs=w_obs,
            w_info=self.w_info,
            w_visit=self.w_visit,
            min_goal_distance=min_goal_distance
            if min_goal_distance is not None else self.min_goal_distance,
            max_obstacle_density=max_obstacle_density
            if max_obstacle_density is not None else self.max_obstacle_density,
            blacklist_radius=blacklist_radius
            if blacklist_radius is not None else self.blacklist_radius,
            hard_blacklist_radius=hard_blacklist_radius
            if hard_blacklist_radius is not None else self.hard_blacklist_radius,
            map_margin_cells=self.map_margin_cells,
            min_clearance_radius_cells=min_clearance_radius_cells
            if min_clearance_radius_cells is not None else self.min_clearance_radius_cells,
            obstacle_radius_cells=self.obstacle_density_radius_cells,
            distance_reward_cap_m=self.distance_reward_cap_m,
            rejection_stats=rejection_stats,
        )

    def select_goal_with_fallback(self, frontiers):
        attempts = [
            {'name': 'base', 'params': {}},
            {
                'name': 'loose_clearance',
                'params': {'min_clearance_radius_cells': 1, 'min_goal_distance': 0.60}
            },
            {
                'name': 'loose_obstacle',
                'params': {
                    'min_clearance_radius_cells': 1,
                    'max_obstacle_density': 0.35,
                    'min_goal_distance': 0.50
                }
            },
            {
                'name': 'loose_blacklist',
                'params': {
                    'min_clearance_radius_cells': 1,
                    'max_obstacle_density': 0.40,
                    'hard_blacklist_radius': 0.50,
                    'min_goal_distance': 0.45,
                }
            },
            {
                'name': 'rescue',
                'params': {
                    'min_clearance_radius_cells': 0,
                    'max_obstacle_density': 0.55,
                    'hard_blacklist_radius': 0.35,
                    'min_goal_distance': 0.30,
                }
            },
        ]
        last_attempt_name = None
        last_stats = None
        for attempt in attempts:
            stats = {}
            params = attempt['params']
            goal = self._select_goal(frontiers, rejection_stats=stats, **params)
            if goal is not None:
                return goal
            last_attempt_name = attempt['name']
            last_stats = stats

        if self.last_goal_selected_time_sec is not None:
            if (self._now_sec() - self.last_goal_selected_time_sec) > self.no_goal_relax_after_sec:
                self.get_logger().warning(
                    'No selectable goal for a while. Relax blacklists and retry.'
                )
                self._clear_all_blacklists('no-goal timeout')
                stats = {}
                goal = self._select_goal(
                    frontiers,
                    max_obstacle_density=0.60,
                    min_clearance_radius_cells=0,
                    blacklist_radius=0.0,
                    hard_blacklist_radius=0.0,
                    min_goal_distance=0.25,
                    rejection_stats=stats,
                )
                if goal is None:
                    self._log_goal_rejection_stats('ttl_rescue', stats)
                return goal
        if last_stats is not None:
            total = last_stats.get('frontiers_total', 0)
            blk = last_stats.get('blacklist_exact', 0) + last_stats.get('blacklist_radius', 0)
            if total > 0 and blk >= total:
                cleared = self._clear_all_blacklists('all candidates blocked by blacklist')
                if not cleared:
                    self._log_goal_rejection_stats(last_attempt_name, last_stats)
                    return None
                stats = {}
                goal = self._select_goal(
                    frontiers,
                    max_obstacle_density=0.60,
                    min_clearance_radius_cells=0,
                    blacklist_radius=0.0,
                    hard_blacklist_radius=0.0,
                    min_goal_distance=0.25,
                    rejection_stats=stats,
                )
                if goal is not None:
                    return goal
                self._log_goal_rejection_stats('blk_saturated_retry', stats)
        if last_stats is not None:
            self._log_goal_rejection_stats(last_attempt_name, last_stats)
        return None

    def _log_goal_rejection_stats(self, attempt_name, stats):
        if not stats:
            return
        self.get_logger().warning(
            'Goal selection failed '
            f'[{attempt_name}] total={stats.get("frontiers_total", 0)}, '
            f'offset={stats.get("offset_invalid", 0)}, '
            f'obs={stats.get("obstacle_density", 0)}, '
            f'near={stats.get("too_near", 0)}, '
            f'blk={stats.get("blacklist_exact", 0) + stats.get("blacklist_radius", 0)}, '
            f'hblk={stats.get("hard_blacklist_exact", 0) + stats.get("hard_blacklist_radius", 0)}, '
            f'margin={stats.get("map_margin", 0)}'
        )

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
        self.goal_start_pose = (self.robot_x, self.robot_y)
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
                self._add_blacklist(key)
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
        # Guard against false-positive success where nav reports success but robot
        # either barely moved or still remains far from the goal.
        if (
            self.last_status == GoalStatus.STATUS_SUCCEEDED
            and self.current_goal is not None
            and self.goal_start_pose is not None
            and self.robot_x is not None
        ):
            sx, sy = self.goal_start_pose
            gx, gy = self.current_goal
            planned = ((gx - sx) ** 2 + (gy - sy) ** 2) ** 0.5
            moved = ((self.robot_x - sx) ** 2 + (self.robot_y - sy) ** 2) ** 0.5
            remaining = ((gx - self.robot_x) ** 2 + (gy - self.robot_y) ** 2) ** 0.5
            if (
                planned >= self.false_success_min_planned_m
                and (
                    moved < self.false_success_min_moved_m
                    or remaining > self.goal_reached_tolerance_m
                )
            ):
                self.get_logger().warning(
                    'Goal reported succeeded but considered invalid '
                    f'(planned={planned:.2f}m, moved={moved:.2f}m, remaining={remaining:.2f}m). '
                    'Treat as failed.'
                )
                self.last_status = GoalStatus.STATUS_ABORTED
        self.goal_active = False
        self.goal_handle = None
        self.goal_sent_time_sec = None
        self.goal_start_pose = None
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
                self._add_blacklist(key)
                self._add_hard_blacklist(key)
        else:
            self.get_logger().warning(f'Goal cancel rejected ({reason}).')
            self.last_status = GoalStatus.STATUS_ABORTED
        self.goal_active = False
        self.goal_handle = None
        self.goal_sent_time_sec = None
        self.goal_start_pose = None
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
