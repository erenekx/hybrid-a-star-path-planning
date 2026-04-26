from astar import astar, astar_with_penalty
from distance_map import compute_distance_map
from visualization import visualize, animate_path
from carla_client import CarlaBridge

import time
import math
import numpy as np


def create_maze_map():
    grid = np.zeros((25, 25))

    grid[0, :] = 1
    grid[-1, :] = 1
    grid[:, 0] = 1
    grid[:, -1] = 1

    grid[2:20, 5] = 1
    grid[5:23, 10] = 1
    grid[2:18, 15] = 1
    grid[10:24, 20] = 1

    grid[8, 5:15] = 1
    grid[15, 10:20] = 1
    grid[18, 3:12] = 1

    # geçitler
    grid[10, 5] = 0
    grid[12, 10] = 0
    grid[6, 15] = 0
    grid[20, 20] = 0

    start = (2, 2)
    goal = (22, 22)

    return grid, start, goal


def compute_metrics(path, distance_map):
    if not path:
        return 0, 0

    length = sum(
        math.hypot(path[i][0] - path[i - 1][0],
                   path[i][1] - path[i - 1][1])
        for i in range(1, len(path))
    )

    min_dist = min(distance_map[x][y] for x, y in path)

    return length, min_dist


def add_theta_to_path(path):
    new_path = []

    for i in range(len(path) - 1):
        x1, y1 = path[i]
        x2, y2 = path[i + 1]

        theta = math.atan2(x2 - x1, y2 - y1)
        new_path.append((x1, y1, theta))

    new_path.append((*path[-1], new_path[-1][2]))
    return new_path


def smooth_path_safe(path, factor=3):
    smooth = []

    for i in range(len(path) - 1):
        x1, y1, t1 = path[i]
        x2, y2, t2 = path[i + 1]

        for t in np.linspace(0, 1, factor):
            x = x1 + (x2 - x1) * t
            y = y1 + (y2 - y1) * t
            theta = t1 + (t2 - t1) * t

            smooth.append((x, y, theta))

    smooth.append(path[-1])
    return smooth


def apply_speed_control(path):
    new_path = []

    for i in range(1, len(path) - 1):
        x_prev, y_prev, _ = path[i - 1]
        x, y, theta = path[i]
        x_next, y_next, _ = path[i + 1]

        v1 = np.array([x - x_prev, y - y_prev])
        v2 = np.array([x_next - x, y_next - y])

        if np.linalg.norm(v1) == 0 or np.linalg.norm(v2) == 0:
            angle = 0
        else:
            cos_angle = np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2))
            angle = np.arccos(np.clip(cos_angle, -1, 1))

        new_path.append((x, y, theta, angle))

    new_path.insert(0, (*path[0], 0))
    new_path.append((*path[-1], 0))

    return new_path


# -------$$--------- MAIN ---------$$-------

grid, start, goal = create_maze_map()
distance_map = compute_distance_map(grid)

print("Start:", start)
print("Goal:", goal)


# BASELINE
t0 = time.time()
path_baseline = astar(grid, start, goal)
t1 = time.time()

print("Baseline A* Time:", t1 - t0)

visualize(grid, path_baseline, start, goal, title="Baseline A*")


# PENALTY
t2 = time.time()
path_penalty = astar_with_penalty(
    grid, start, goal, distance_map, lambda_weight=15)
t3 = time.time()

print("Penalty A* Time:", t3 - t2)

# when you need graphics based on 2D Map, use this visualization. (also disabled for matplotlib error.)
# visualize(grid, path_penalty, start, goal, title="Obstacle Aware Path")


# ⚠️IMPORTANT: We are providing the RAW PATH for dynamic replanning.
animate_path(grid, path_penalty, start, goal)

print("\n---$$ Initializing CARLA 3D Environment $$---")

path_with_theta = add_theta_to_path(path_penalty)
smoothed_carla_path = smooth_path_safe(path_with_theta, factor=2)

# Harita ofset değerlerini bir kere tanımla (Eğer araba binalara çarpıyorsa bu sayıları değiştir: örn 50, 50 yap)
X_OFFSET = 49.0
Y_OFFSET = 49.0

bridge = CarlaBridge(cell_size=3)

if bridge.connect():
    # 1. Önce 2D labirentimizi 3D konilerle CARLA'ya inşa et!
    bridge.build_custom_maze(grid, offset_x=X_OFFSET, offset_y=Y_OFFSET)

    # 2. Aracı bu labirentin başlangıç koordinatına koy
    bridge.spawn_vehicle(start, offset_x=X_OFFSET, offset_y=Y_OFFSET)

    # 3. Yolu takip et (Sensörsüz, sadece matematik ile)
    bridge.drive_path(smoothed_carla_path, offset_x=X_OFFSET, offset_y=Y_OFFSET)

    # İşimiz bitince her şeyi silmek için:
    time.sleep(5)
    bridge.cleanup()