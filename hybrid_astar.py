import numpy as np
import math
import heapq

# Vehicle and Grid Configuration
L = 2.7  # Wheelbase
MAX_STEER = 0.6  # Max steering angle in radians (approx 35 deg)
XY_RES = 0.5  # Spatial resolution in meters
YAW_RES = np.deg2rad(15.0)  # Angular resolution (15 degrees)


class Node:
    def __init__(self, x_idx, y_idx, yaw_idx, x, y, yaw, direction, steer, parent_index, cost, time=0.0):
        self.x_idx = x_idx
        self.y_idx = y_idx
        self.yaw_idx = yaw_idx
        self.x = x
        self.y = y
        self.yaw = yaw
        self.direction = direction
        self.steer = steer
        self.parent_index = parent_index
        self.cost = cost
        self.time = time  # Tracks when the vehicle reaches this state


def bicycle_model(x, y, yaw, steer, L, v, dt):
    """Updates vehicle state based on Kinematic Bicycle Model."""
    x += v * math.cos(yaw) * dt
    y += v * math.sin(yaw) * dt
    yaw += (v / L) * math.tan(steer) * dt
    yaw = (yaw + math.pi) % (2 * math.pi) - math.pi
    return x, y, yaw


def is_collision(node, grid, dynamic_obstacles=None):
    """Checks vehicle footprint against static and dynamic obstacles."""
    # 1. Footprint Dimensions
    W = 2.0
    L_front = 3.5
    L_back = 1.0

    # 2. Static Obstacle Check (Grid based)
    check_points = [(L_front, W / 2), (L_front, -W / 2), (-L_back, W / 2), (-L_back, -W / 2), (0, 0)]

    for dx, dy in check_points:
        check_x = node.x + dx * math.cos(node.yaw) - dy * math.sin(node.yaw)
        check_y = node.y + dx * math.sin(node.yaw) + dy * math.cos(node.yaw)

        idx_x = int(round(check_x / 2.5))  # Scaling to your 2.5m grid cells
        idx_y = int(round(check_y / 2.5))

        if idx_x < 0 or idx_x >= grid.shape[0] or idx_y < 0 or idx_y >= grid.shape[1]:
            return True
        if grid[idx_x, idx_y] >= 1.0:
            return True

    # 3. Dynamic Obstacle Check (Time-Distance based)
    if dynamic_obstacles:
        for obs in dynamic_obstacles:
            pred_x = obs['x'] + obs['vx'] * node.time
            pred_y = obs['y'] + obs['vy'] * node.time

            dist = math.hypot(node.x - pred_x, node.y - pred_y)
            if dist < obs['radius']:
                return True

    return False


def get_successors(current_node, dt, dynamic_obstacles=None):
    """Generates neighbors including time evolution."""
    successors = []
    steer_inputs = [-MAX_STEER, 0.0, MAX_STEER]

    for steer in steer_inputs:
        new_x, new_y, new_yaw = bicycle_model(current_node.x, current_node.y, current_node.yaw, steer, L, 1.5, dt)
        new_time = current_node.time + dt

        new_node = Node(
            int(round(new_x / XY_RES)), int(round(new_y / XY_RES)), int(round(new_yaw / YAW_RES)),
            new_x, new_y, new_yaw, 1, steer, None,
            current_node.cost + dt + abs(steer) * 0.2,
            time=new_time
        )
        successors.append(new_node)
    return successors


def hybrid_astar_planning(start, goal, grid, dynamic_obstacles=None, dt=0.5):
    """Hybrid A* search loop supporting dynamic obstacles."""
    start_node = Node(int(round(start[0] / XY_RES)), int(round(start[1] / XY_RES)), int(round(start[2] / YAW_RES)),
                      start[0], start[1], start[2], 1, 0.0, None, 0.0, time=0.0)

    open_set = {(start_node.x_idx, start_node.y_idx, start_node.yaw_idx): start_node}
    closed_set = {}
    pq = [(start_node.cost, (start_node.x_idx, start_node.y_idx, start_node.yaw_idx))]

    while pq:
        _, current_idx = heapq.heappop(pq)
        if current_idx not in open_set: continue

        current_node = open_set.pop(current_idx)
        closed_set[current_idx] = current_node

        if math.hypot(current_node.x - goal[0], current_node.y - goal[1]) < 2.0:
            return reconstruct_path(current_node, closed_set)

        for neighbor in get_successors(current_node, dt, dynamic_obstacles):
            if is_collision(neighbor, grid, dynamic_obstacles): continue

            n_idx = (neighbor.x_idx, neighbor.y_idx, neighbor.yaw_idx)
            if n_idx in closed_set: continue

            if n_idx not in open_set or open_set[n_idx].cost > neighbor.cost:
                neighbor.parent_index = current_idx
                open_set[n_idx] = neighbor
                h = math.hypot(neighbor.x - goal[0], neighbor.y - goal[1])
                heapq.heappush(pq, (neighbor.cost + h, n_idx))
    return None


def reconstruct_path(final_node, closed_set):
    px, py, pyaw, pt = [], [], [], []
    curr = final_node
    while curr is not None:
        px.append(curr.x);
        py.append(curr.y);
        pyaw.append(curr.yaw);
        pt.append(curr.time)
        curr = closed_set.get(curr.parent_index)
    return px[::-1], py[::-1], pyaw[::-1], pt[::-1]