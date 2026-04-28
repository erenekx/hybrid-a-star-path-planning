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

    # geçitler: 3 hücre genişliğinde açıldı (3 x 2.5m = 7.5m)
    # Tesla Model 3 genişliği: 1.85m → 7.5m geniş geçit → rahat geçer
    grid[9:12, 5] = 0    # was: grid[10, 5]   → tek hücre (2.5m) → çok dar
    grid[11:14, 10] = 0  # was: grid[12, 10]  → tek hücre (2.5m) → çok dar
    grid[5:8, 15] = 0    # was: grid[6, 15]   → tek hücre (2.5m) → çok dar
    grid[19:22, 20] = 0  # was: grid[20, 20]  → tek hücre (2.5m) → çok dar

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

# =============================================================
# YENİ YAKLAŞIM: Yol tabanlı koridor + road spawn
# =============================================================
#
# START_LOCATION → Rotanın başladığı/bittiği nokta (kırmızı çizgilerin kesişimi)
# Koordinatları CARLA'da spectator ile bulabilirsin:
#   spectator = world.get_spectator()
#   print(spectator.get_transform().location)
#
# Town05 için başlangıç noktası — bunu kendi haritana göre ayarla:
import carla as _carla
START_LOCATION = _carla.Location(x=0.0, y=0.0, z=0.0)   # ← BURAYA GERÇEK KOORDİNATI YAZ
ROUTE_DISTANCE = 200.0   # Rota boyunca kaç metre gidileceği

bridge = CarlaBridge(cell_size=2.5)

if bridge.connect():
    # 1. Başlangıç noktasından itibaren CARLA yol waypoint'leri üret
    route_wps = bridge.get_road_route(
        start_location=START_LOCATION,
        total_distance=ROUTE_DISTANCE,
        step=2.0
    )

    if route_wps:
        # 2. Konileri yolun KENARINA diz (araç ortadan geçer)
        bridge.build_road_corridor(
            route_waypoints=route_wps,
            cone_spacing=3.5,   # koniler arası mesafe (m)
            cone_offset=4.5     # yol merkezinden yan mesafe (m) → toplam genişlik: 9m
        )

        # 3. Aracı yola snap et (roundabout/fıskiye'ye düşmez)
        bridge.spawn_vehicle_on_road(START_LOCATION)

        # 4. Yol waypoint'leri boyunca süz
        bridge.drive_waypoints(route_wps, arrival_threshold=3.0)

    # Temizlik
    time.sleep(5)
    bridge.cleanup()