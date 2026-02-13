import math
from collections import deque


def world_to_grid(map_msg, wx, wy):
    ox = map_msg.info.origin.position.x
    oy = map_msg.info.origin.position.y
    res = map_msg.info.resolution
    gx = int((wx - ox) / res)
    gy = int((wy - oy) / res)
    return gx, gy


def grid_to_world(map_msg, gx, gy):
    ox = map_msg.info.origin.position.x
    oy = map_msg.info.origin.position.y
    res = map_msg.info.resolution
    wx = ox + (gx + 0.5) * res
    wy = oy + (gy + 0.5) * res
    return wx, wy


def obstacle_density(map_msg, wx, wy, radius_cells=6):
    gx, gy = world_to_grid(map_msg, wx, wy)
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


def unknown_gain(map_msg, wx, wy, radius_cells=6):
    gx, gy = world_to_grid(map_msg, wx, wy)
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


def is_within_map_margin(map_msg, wx, wy, margin_cells):
    if margin_cells <= 0:
        return True
    gx, gy = world_to_grid(map_msg, wx, wy)
    w = map_msg.info.width
    h = map_msg.info.height
    return (
        margin_cells <= gx < (w - margin_cells)
        and margin_cells <= gy < (h - margin_cells)
    )


def has_clearance_from_obstacles(map_msg, wx, wy, radius_cells):
    if radius_cells <= 0:
        return True
    gx, gy = world_to_grid(map_msg, wx, wy)
    w = map_msg.info.width
    h = map_msg.info.height
    data = map_msg.data
    for dy in range(-radius_cells, radius_cells + 1):
        for dx in range(-radius_cells, radius_cells + 1):
            x = gx + dx
            y = gy + dy
            if x < 0 or x >= w or y < 0 or y >= h:
                return False
            v = data[y * w + x]
            if v > 50:
                return False
    return True


def is_free_cell(map_msg, wx, wy):
    gx, gy = world_to_grid(map_msg, wx, wy)
    w = map_msg.info.width
    h = map_msg.info.height
    if gx < 0 or gx >= w or gy < 0 or gy >= h:
        return False
    return map_msg.data[gy * w + gx] == 0


def compute_reachable_free_cells(map_msg, start_wx, start_wy):
    w = map_msg.info.width
    h = map_msg.info.height
    data = map_msg.data
    sx, sy = world_to_grid(map_msg, start_wx, start_wy)
    if sx < 0 or sx >= w or sy < 0 or sy >= h:
        return set()
    if data[sy * w + sx] != 0:
        return set()

    reachable = set()
    queue = deque([(sx, sy)])
    reachable.add((sx, sy))
    while queue:
        cx, cy = queue.popleft()
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nx = cx + dx
            ny = cy + dy
            if nx < 0 or nx >= w or ny < 0 or ny >= h:
                continue
            if (nx, ny) in reachable:
                continue
            if data[ny * w + nx] != 0:
                continue
            reachable.add((nx, ny))
            queue.append((nx, ny))
    return reachable


def offset_goal_toward_robot(
    map_msg,
    frontier_x,
    frontier_y,
    robot_x,
    robot_y,
    reachable,
    map_margin_cells,
    min_clearance_radius_cells,
    start_offset=0.20,
    max_offset=0.45,
    step=0.05,
):
    d = math.hypot(robot_x - frontier_x, robot_y - frontier_y)
    if d < 1e-6:
        return None
    ux = (robot_x - frontier_x) / d
    uy = (robot_y - frontier_y) / d

    offset = start_offset
    while offset <= max_offset + 1e-6:
        gx = frontier_x + ux * offset
        gy = frontier_y + uy * offset
        if not is_within_map_margin(map_msg, gx, gy, map_margin_cells):
            offset += step
            continue
        if not is_free_cell(map_msg, gx, gy):
            offset += step
            continue
        if not has_clearance_from_obstacles(map_msg, gx, gy, min_clearance_radius_cells):
            offset += step
            continue
        gix, giy = world_to_grid(map_msg, gx, gy)
        if (gix, giy) not in reachable:
            offset += step
            continue
        return gx, gy
    return None


def extract_frontiers(map_msg, min_cluster_size):
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

    return cluster_frontier_cells(map_msg, frontier_cells, min_cluster_size)


def cluster_frontier_cells(map_msg, frontier_cells, min_cluster_size):
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

        if len(cluster) < min_cluster_size:
            continue

        mean_x = sum(c[0] for c in cluster) / len(cluster)
        mean_y = sum(c[1] for c in cluster) / len(cluster)
        wx, wy = grid_to_world(map_msg, mean_x, mean_y)
        clustered_world_points.append((wx, wy))

    return clustered_world_points


def select_goal(
    frontiers,
    map_msg,
    robot_x,
    robot_y,
    blacklist,
    hard_blacklist,
    visited_count,
    phase,
    rich_min_density,
    w_dist,
    w_obs,
    w_info,
    w_visit,
    min_goal_distance=0.35,
    max_obstacle_density=0.45,
    blacklist_radius=0.0,
    hard_blacklist_radius=0.0,
    map_margin_cells=0,
    min_clearance_radius_cells=0,
):
    best = None
    best_score = -1e18
    reachable = compute_reachable_free_cells(map_msg, robot_x, robot_y)
    for fx, fy in frontiers:
        nav_goal = offset_goal_toward_robot(
            map_msg=map_msg,
            frontier_x=fx,
            frontier_y=fy,
            robot_x=robot_x,
            robot_y=robot_y,
            reachable=reachable,
            map_margin_cells=map_margin_cells,
            min_clearance_radius_cells=min_clearance_radius_cells,
        )
        if nav_goal is None:
            continue
        gx, gy = nav_goal
        key = (round(gx, 2), round(gy, 2))
        if key in blacklist:
            continue
        if key in hard_blacklist:
            continue
        if hard_blacklist_radius > 0.0:
            near_hard_blacklisted = any(
                math.hypot(gx - bx, gy - by) <= hard_blacklist_radius
                for bx, by in hard_blacklist
            )
            if near_hard_blacklisted:
                continue
        if not is_within_map_margin(map_msg, fx, fy, map_margin_cells):
            continue
        if blacklist_radius > 0.0:
            near_blacklisted = any(
                math.hypot(gx - bx, gy - by) <= blacklist_radius
                for bx, by in blacklist
            )
            if near_blacklisted:
                continue

        d = math.hypot(gx - robot_x, gy - robot_y)
        if d < min_goal_distance:
            continue

        obs = obstacle_density(map_msg, fx, fy, radius_cells=6)
        if obs > max_obstacle_density:
            continue

        info = unknown_gain(map_msg, fx, fy, radius_cells=6)
        v = visited_count.get(key, 0)
        score = w_info * info - w_dist * d - w_obs * obs - w_visit * v

        if phase == 'RICH' and obs < rich_min_density:
            continue

        if score > best_score:
            best_score = score
            best = (gx, gy)

    return best
