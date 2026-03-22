"""
Purpose of Main.py:
This is the main executable file of the project. It acts as the core center:
creates maps via user terminal input, runs algorithms, performs Lambda sensitivity analysis,
and generates comprehensive scalability/performance graphs.
"""
from grid_map import create_test_map, create_maze_map, create_narrow_corridor_map, create_random_clutter_map
from astar import astar, astar_with_penalty
from distance_map import compute_distance_map
from visualization import visualize, visualize_distance
import time
import matplotlib.pyplot as plt
import math

def compute_metrics(path, distance_map):
    if path is None:
        return 0, 0
    length = 0
    for i in range(1, len(path)):
        x1, y1 = path[i-1]
        x2, y2 = path[i]
        length += math.hypot(x2-x1, y2-y1)
    min_dist = min(distance_map[x][y] for x, y in path)
    return length, min_dist

# ==========================================
# 1. ENVIRONMENT PREPARATION & MAP SELECTION
# ==========================================
print("\n" + "="*40)
print("AUTONOMOUS VEHICLE PATH PLANNING")
print("="*40)
print("Please(Lütfen), (harita seçin)choose the map you want to use: ")
print("1 - Baseline Map (Temel Test Haritası)")
print("2 - Maze (Labirent - Çıkmaz Sokak Testi)")
print("3 - Narrow Corridor (Dar Geçit - Merkezleme Testi)")
print("4 - Random Clutter (Rastgele %30 Engel Yoğunluklu Harita)")
print("="*40)

while True:
    choice = input("Seçiminiz (1/2/3/4): ")
    if choice == '1':
        grid, start, goal = create_test_map()
        print("\n--> Baseline Map chosen. Starting...")
        break
    elif choice == '2':
        grid, start, goal = create_maze_map()
        print("\n--> Maze (Labirent) chosen. Starting....")
        break
    elif choice == '3':
        grid, start, goal = create_narrow_corridor_map()
        print("\n--> Narrow Corridor (Dar Geçit) chosen. Starting...")
        break
    elif choice == '4':
        grid, start, goal = create_random_clutter_map(size=30, obstacle_density=0.3)
        print("\n--> Random Clutter Map chosen. Starting...")
        break
    else:
        print("Invalid(Geçersiz)! Please choose one of 1, 2, 3 or 4 numbers to proceed.")

#time complexity O(1) PRECOMPUTATION: Distance map is calculated ONLY ONCE here based on user selection.
distance_map = compute_distance_map(grid)
visualize_distance(distance_map)

# ===================================
# 2. RUNNING ALGORITHMS (BASELINE)
# ==================================
start_time = time.time()
path_baseline = astar(grid, start, goal)
baseline_time = time.time() - start_time

print(f"\nBaseline A* Time: {baseline_time:.4f}s")
visualize(grid, path_baseline, start, goal, title=f"Standard A* (Baseline) - Map {choice}")

# ================================
# 3. LAMBDA SENSITIVITY ANALYSIS
# ================================
lambda_values = [0, 2, 5, 8, 15]
lambda_results = []

print("\n--- Running Lambda Analysis ---")
for lw in lambda_values:
    st = time.time()
    path_pen = astar_with_penalty(grid, start, goal, distance_map, lambda_weight=lw)
    pt = time.time() - st
    plen, pmin = compute_metrics(path_pen, distance_map)
    lambda_results.append((lw, pt, plen, pmin))
    print(f"Lambda {lw}: Time={pt:.4f}s, Path Length={plen:.2f}, Min Dist={pmin}")

# ==========================================
# 4. SCALABILITY & DENSITY PERFORMANCE TESTS (WEEK 4)
# ==========================================
print("\n--- Running Scalability Test (Map Size vs Time) ---")
map_sizes = [10, 20, 30, 40, 50]
size_times = []

for size in map_sizes:
    t_grid, t_start, t_goal = create_random_clutter_map(size=size, obstacle_density=0.2)
    t_dist_map = compute_distance_map(t_grid)
    st = time.time()
    astar_with_penalty(t_grid, t_start, t_goal, t_dist_map, lambda_weight=5)
    size_times.append(time.time() - st)

print("--- Running Density Test (Obstacle % vs Time) ---")
densities = [0.1, 0.2, 0.3, 0.4]
density_times = []
constant_size = 30

for density in densities:
    d_grid, d_start, d_goal = create_random_clutter_map(size=constant_size, obstacle_density=density)
    d_dist_map = compute_distance_map(d_grid)
    st = time.time()
    astar_with_penalty(d_grid, d_start, d_goal, d_dist_map, lambda_weight=5)
    density_times.append(time.time() - st)

# ==================================
# 5. PLOTTING ALL GRAPHS (DASHBOARD)
# ==================================
print("\nCompleted. Plotting graphs...")
plt.figure(figsize=(15, 10))

# Graph 1: Lambda vs Min Obstacle Distance
plt.subplot(2, 3, 1)
plt.plot([r[0] for r in lambda_results], [r[3] for r in lambda_results], marker='o', color='b')
plt.title("Safety: Lambda vs Min Distance")
plt.xlabel("Lambda Weight")
plt.ylabel("Min Distance to Obstacle")
plt.grid(True)

# Graph 2: Lambda vs Path Length
plt.subplot(2, 3, 2)
plt.plot([r[0] for r in lambda_results], [r[2] for r in lambda_results], marker='o', color='g')
plt.title("Cost: Lambda vs Path Length")
plt.xlabel("Lambda Weight")
plt.ylabel("Total Path Length")
plt.grid(True)

# Graph 3: Lambda vs Computation Time
plt.subplot(2, 3, 3)
plt.plot([r[0] for r in lambda_results], [r[1] for r in lambda_results], marker='o', color='r')
plt.title("Performance: Lambda vs Time")
plt.xlabel("Lambda Weight")
plt.ylabel("Time (seconds)")
plt.grid(True)

# Graph 4: Map Size vs Computation Time (Scalability)
plt.subplot(2, 3, 4)
plt.plot(map_sizes, size_times, marker='s', color='purple', linestyle='--')
plt.title("(Scalblty)M. Size vs Time7u89")
plt.xlabel("Map Size (N x N)")
plt.ylabel("Computation Time (s)")
plt.grid(True)

# Graph 5: Obstacle Density vs Computation Time
plt.subplot(2, 3, 5)
plt.plot([d * 100 for d in densities], density_times, marker='^', color='orange', linestyle='-.')
plt.title("(Complxty)Obs. Density vs Time")
plt.xlabel("Obstacle Density (%)")
plt.ylabel("Computation Time (s)")
plt.grid(True)

plt.subplots_adjust(left=0.10, right=0.95, bottom=0.10, top=0.92, hspace=0.7, wspace=0.3)
plt.show()

# Final Visualization of the Safe Route
safe_path = astar_with_penalty(grid, start, goal, distance_map, lambda_weight=8)
visualize(grid, safe_path, start, goal, title=f"Penalty Factor A* (Lambda=8) - Map {choice}")