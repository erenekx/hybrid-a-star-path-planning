import time
import math
import numpy as np
import matplotlib.pyplot as plt

# Safe import for CARLA
try:
    import carla as _carla

    CARLA_AVAILABLE = True
except ImportError:
    CARLA_AVAILABLE = False
    print("\n[!] CARLA library not found. Running in Algorithm-Only mode.")

from astar import astar, astar_with_penalty
from distance_map import compute_distance_map
from visualization import visualize, animate_path
from carla_client import CarlaBridge
from hybrid_astar import hybrid_astar_planning


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
    grid[9:12, 5] = 0
    grid[11:14, 10] = 0
    grid[5:8, 15] = 0
    grid[19:22, 20] = 0

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
def main():
    grid, start, goal = create_maze_map()
    distance_map = compute_distance_map(grid)

    print("Start (Grid):", start)
    print("Goal (Grid):", goal)

    # 1. BASELINE A*
    print("\n--- Running Baseline A* ---")
    t0 = time.time()
    path_baseline = astar(grid, start, goal)
    print("Baseline A* Time:", time.time() - t0)
    visualize(grid, path_baseline, start, goal, title="Baseline A*")

    # 2. PENALTY A*
    print("\n--- Running Penalty A* ---")
    t2 = time.time()
    path_penalty = astar_with_penalty(grid, start, goal, distance_map, lambda_weight=15)
    print("Penalty A* Time:", time.time() - t2)

    print("Animating Penalty Path...")
    animate_path(grid, path_penalty, start, goal)

    # 3. HYBRID A* (Dynamic Obstacles)
    CELL_SIZE = 2.5
    start_hybrid = [start[0] * CELL_SIZE, start[1] * CELL_SIZE, 0.0]
    goal_hybrid = [goal[0] * CELL_SIZE, goal[1] * CELL_SIZE, 0.0]

    moving_objects = [
        {
            'x': 25.0,
            'y': 15.0,
            'vx': 0.0,
            'vy': 1.0,
            'radius': 2.5
        }
    ]

    print("\n--- Running Hybrid A* Planning with Dynamic Obstacles ---")
    t_h = time.time()
    hybrid_path = hybrid_astar_planning(start_hybrid, goal_hybrid, grid, dynamic_obstacles=moving_objects, dt=0.5)
    print("Hybrid A* Search Time:", time.time() - t_h)

    if hybrid_path:
        hx, hy, hyaw, ht = hybrid_path
        print(f"Hybrid Path found! Nodes: {len(hx)}")

        plt.figure(figsize=(10, 10))
        plt.imshow(grid.T, cmap='Greys', origin='lower',
                   extent=[0, grid.shape[0] * CELL_SIZE, 0, grid.shape[1] * CELL_SIZE])
        plt.plot(hx, hy, "-r", linewidth=2, label="Hybrid A* Path")
        plt.scatter(start_hybrid[0], start_hybrid[1], c='g', s=100, label="Start")
        plt.scatter(goal_hybrid[0], goal_hybrid[1], c='b', s=100, label="Goal")
        plt.scatter(moving_objects[0]['x'], moving_objects[0]['y'], c='orange', marker='X', s=150,
                    label="Dynamic Obstacle (Start)")
        plt.title("Hybrid A* (Kinematic + Dynamic Obstacle Avoidance)")
        plt.legend()
        plt.grid(True, alpha=0.3)
        plt.show()

        # 4. CARLA INTEGRATION
        if CARLA_AVAILABLE:
            print("\n---$$ Initializing CARLA 3D Environment $$---")
            START_LOCATION = _carla.Location(x=0.0, y=0.0, z=0.0)
            ROUTE_DISTANCE = 200.0

            bridge = CarlaBridge(cell_size=CELL_SIZE)

            if bridge.connect():
                route_wps = bridge.get_road_route(
                    start_location=START_LOCATION,
                    total_distance=ROUTE_DISTANCE,
                    step=2.0
                )

                if route_wps:
                    bridge.build_road_corridor(
                        route_waypoints=route_wps,
                        cone_spacing=3.5,
                        cone_offset=4.5
                    )
                    bridge.spawn_vehicle_on_road(START_LOCATION)
                    bridge.drive_waypoints(route_wps, arrival_threshold=3.0)

                time.sleep(5)
                bridge.cleanup()
    else:
        print("CRITICAL: Hybrid A* could not find a path.")


if __name__ == '__main__':
    main()