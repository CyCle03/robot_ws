import math
from collections import deque
from enum import Enum

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile


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
        qos = QoSProfile(depth=10)

        self.map_msg = None
        self.robot_x = None
        self.robot_y = None
        self.state = ExplorerState.IDLE
        self.phase = Phase.RICH
        self.goal_active = False
        self.goal_handle = None
        self.goal_sent_time_sec = None
        self.current_goal = None
        self.blacklist = set()
        self.visited_count = {}
        self.goal_fail_count = {}
        self.last_result = None
        self.last_status = None

        self.rich_min_density = 0.10
        self.goal_timeout_sec = 60.0
        self.max_goal_retries = 2
        self.frontier_min_cluster_size = 10
        self.w_dist = 1.0
        self.w_obs = 1.5
        self.w_info = 1.0
        self.w_visit = 0.8

        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, qos)
        self.create_subscription(Odometry, '/odom', self.odom_callback, qos)
        self.timer = self.create_timer(1.0, self.step)

    def map_callback(self, msg):
        self.map_msg = msg

    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

    def step(self):
        if self.map_msg is None or self.robot_x is None:
            return

        if self.state == ExplorerState.IDLE:
            self.state = ExplorerState.SELECT_GOAL
            return

        if self.state == ExplorerState.SELECT_GOAL:
            frontiers = self.extract_frontiers(self.map_msg)
            if not frontiers:
                if self.phase == Phase.RICH:
                    self.phase = Phase.COVERAGE
                    self.get_logger().info('Switch phase: RICH -> COVERAGE')
                    return
                self.state = ExplorerState.DONE
                self.get_logger().info('No frontiers left. Exploration done.')
                return

            goal = self.select_goal(frontiers)
            if goal is None:
                if self.phase == Phase.RICH:
                    self.phase = Phase.COVERAGE
                    self.get_logger().info('No rich frontier. Switch to COVERAGE.')
                else:
                    self.state = ExplorerState.DONE
                return

            self.send_nav_goal(goal)
            self.current_goal = goal
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

            self.current_goal = None
            self.last_result = None
            self.last_status = None
            self.state = ExplorerState.SELECT_GOAL

    def _now_sec(self):
        return self.get_clock().now().nanoseconds / 1e9

    def extract_frontiers(self, map_msg):
        w = map_msg.info.width
        h = map_msg.info.height
        data = map_msg.data
        frontier_cells = []

        def idx(x, y):
            return y * w + x

        for y in range(1, h - 1):
            for x in range(1, w - 1):
                if data[idx(x, y)] != 0:
                    continue
                unknown_neighbor = False
                for ny in (-1, 0, 1):
                    for nx in (-1, 0, 1):
                        if nx == 0 and ny == 0:
                            continue
                        if data[idx(x + nx, y + ny)] == -1:
                            unknown_neighbor = True
                            break
                    if unknown_neighbor:
                        break
                if unknown_neighbor:
                    frontier_cells.append((x, y))

        return self.cluster_frontier_cells(map_msg, frontier_cells)

    def cluster_frontier_cells(self, map_msg, frontier_cells):
        frontier_set = set(frontier_cells)
        visited = set()
        clustered_world_points = []

        for cell in frontier_cells:
            if cell in visited:
                continue
            queue = deque([cell])
            visited.add(cell)
            cluster = []

            while queue:
                cx, cy = queue.popleft()
                cluster.append((cx, cy))
                for ny in (-1, 0, 1):
                    for nx in (-1, 0, 1):
                        if nx == 0 and ny == 0:
                            continue
                        nb = (cx + nx, cy + ny)
                        if nb in frontier_set and nb not in visited:
                            visited.add(nb)
                            queue.append(nb)

            if len(cluster) < self.frontier_min_cluster_size:
                continue

            mean_x = sum(c[0] for c in cluster) / len(cluster)
            mean_y = sum(c[1] for c in cluster) / len(cluster)
            wx, wy = self.grid_to_world(map_msg, mean_x, mean_y)
            clustered_world_points.append((wx, wy))

        return clustered_world_points

    def select_goal(self, frontiers):
        best = None
        best_score = -1e18
        for fx, fy in frontiers:
            key = (round(fx, 2), round(fy, 2))
            if key in self.blacklist:
                continue

            d = math.hypot(fx - self.robot_x, fy - self.robot_y)
            obs = self.obstacle_density(self.map_msg, fx, fy, radius_cells=6)
            info = self.unknown_gain(self.map_msg, fx, fy, radius_cells=6)
            v = self.visited_count.get(key, 0)
            score = self.w_obs * obs + self.w_info * info - self.w_dist * d - self.w_visit * v

            if self.phase == Phase.RICH and obs < self.rich_min_density:
                continue

            if score > best_score:
                best_score = score
                best = (fx, fy)

        return best

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
                self.blacklist.add((round(self.current_goal[0], 2), round(self.current_goal[1], 2)))
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

    def world_to_grid(self, map_msg, wx, wy):
        ox = map_msg.info.origin.position.x
        oy = map_msg.info.origin.position.y
        res = map_msg.info.resolution
        gx = int((wx - ox) / res)
        gy = int((wy - oy) / res)
        return gx, gy

    def grid_to_world(self, map_msg, gx, gy):
        ox = map_msg.info.origin.position.x
        oy = map_msg.info.origin.position.y
        res = map_msg.info.resolution
        wx = ox + (gx + 0.5) * res
        wy = oy + (gy + 0.5) * res
        return wx, wy

    def obstacle_density(self, map_msg, wx, wy, radius_cells=6):
        gx, gy = self.world_to_grid(map_msg, wx, wy)
        w = map_msg.info.width
        h = map_msg.info.height
        data = map_msg.data
        occ = 0
        total = 0
        for dy in range(-radius_cells, radius_cells + 1):
            for dx in range(-radius_cells, radius_cells + 1):
                x = gx + dx
                y = gy + dy
                if x < 0 or x >= w or y < 0 or y >= h:
                    continue
                v = data[y * w + x]
                if v < 0:
                    continue
                total += 1
                if v > 50:
                    occ += 1
        if total == 0:
            return 0.0
        return occ / total

    def unknown_gain(self, map_msg, wx, wy, radius_cells=6):
        gx, gy = self.world_to_grid(map_msg, wx, wy)
        w = map_msg.info.width
        h = map_msg.info.height
        data = map_msg.data
        unk = 0
        total = 0
        for dy in range(-radius_cells, radius_cells + 1):
            for dx in range(-radius_cells, radius_cells + 1):
                x = gx + dx
                y = gy + dy
                if x < 0 or x >= w or y < 0 or y >= h:
                    continue
                total += 1
                if data[y * w + x] == -1:
                    unk += 1
        if total == 0:
            return 0.0
        return unk / total

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
