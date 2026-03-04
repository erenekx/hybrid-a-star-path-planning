from grid_map import create_test_map
from astar import astar, astar_with_penalty
from distance_map import compute_distance_map
from visualization import visualize, visualize_distance
import time
import matplotlib.pyplot as plt


def compute_metrics(path, distance_map):
    import math

    if path is None:
        return 0, 0

    length = 0
    for i in range(1, len(path)):
        x1, y1 = path[i-1]
        x2, y2 = path[i]
        length += math.hypot(x2-x1, y2-y1)

    min_dist = min(distance_map[x][y] for x,y in path)

    return length, min_dist


grid, start, goal = create_test_map()

distance_map = compute_distance_map(grid)
visualize_distance(distance_map)

start_time = time.time()
path_baseline = astar(grid,start,goal)
baseline_time = time.time()-start_time

print("Baseline A* Time:", baseline_time)

baseline_length, baseline_min_dist = compute_metrics(path_baseline,distance_map)

print("Baseline Path Length:",baseline_length)
print("Baseline Min Obstacle Distance:",baseline_min_dist)

visualize(grid,path_baseline,start,goal)


lambda_values=[0,2,5,8,15]
results=[]

for lambda_weight in lambda_values:

    start_time=time.time()

    path_penalty=astar_with_penalty(
        grid,start,goal,distance_map,lambda_weight=lambda_weight
    )

    penalty_time=time.time()-start_time

    penalty_length,penalty_min_dist=compute_metrics(path_penalty,distance_map)

    print("\nLambda:",lambda_weight)
    print("Time:",penalty_time)
    print("Path Length:",penalty_length)
    print("Min Obstacle Distance:",penalty_min_dist)

    results.append((lambda_weight,penalty_time,penalty_length,penalty_min_dist))


lambdas=[r[0] for r in results]
times=[r[1] for r in results]
path_lengths=[r[2] for r in results]
min_distances=[r[3] for r in results]

plt.figure()
plt.plot(lambdas,min_distances,marker='o')
plt.xlabel("Lambda")
plt.ylabel("Min Obstacle Distance")
plt.title("Lambda vs Min Obstacle Distance")
plt.grid(True)
plt.show()

plt.figure()
plt.plot(lambdas,path_lengths,marker='o')
plt.xlabel("Lambda")
plt.ylabel("Path Length")
plt.title("Lambda vs Path Length")
plt.grid(True)
plt.show()

plt.figure()
plt.plot(lambdas,times,marker='o')
plt.xlabel("Lambda")
plt.ylabel("Computation Time")
plt.title("Lambda vs Computation Time")
plt.grid(True)
plt.show()


example_path=astar_with_penalty(grid,start,goal,distance_map,lambda_weight=8)

visualize(grid,example_path,start,goal)