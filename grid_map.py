"""
Purpose of grid_map.py:
Generates various virtual map environments (Basic, Maze, Narrow Corridors,
and Random Clutter) to rigorously test path planning algorithms.
These maps evaluate how the algorithm handles dead-ends, maintains safety
margins in tight spaces, and scales with different obstacle densities.

Grid Values:
0 = Free space
1 = Rough terrain (higher movement cost)
2 = Blocked cell / Obstacle
"""

import numpy as np
import random


def create_test_map():
    # Original basic test map
    grid = np.zeros((20, 20))

    grid[5:15, 10] = 2
    grid[10, 5:15] = 2
    grid[3:8, 3:8] = 1
    grid[12:17, 12:17] = 1

    start = (2, 2)
    goal = (17, 17)

    return grid, start, goal


def create_maze_map():
    grid = np.zeros((20, 20))

    # Creates U-shaped traps and dead-ends to test algorithm backtracking
    grid[4:16, 5] = 2
    grid[15, 5:15] = 2
    grid[4:16, 15] = 2
    grid[9, 5:10] = 2  # Inner wall creating a dead-end

    start = (2, 2)
    goal = (10, 10)  # Placed inside the U-shape

    return grid, start, goal


def create_narrow_corridor_map():
    grid = np.zeros((20, 20))

    # Creates a long, tight corridor bounded by walls.
    # Useful for testing if the Obstacle Penalty forces the path to the center.
    grid[6:8, 2:18] = 2  # Top wall
    grid[11:13, 2:18] = 2  # Bottom wall

    # A bottleneck in the middle
    grid[8:11, 9:11] = 2
    grid[9, 9:11] = 0  # Leaves a 1-cell wide gap

    start = (9, 1)
    goal = (9, 18)

    return grid, start, goal


def create_random_clutter_map(size=20, obstacle_density=0.3):
    grid = np.zeros((size, size))

    start = (1, 1)
    goal = (size - 2, size - 2)

    # Calculate the total number of obstacles based on the requested density
    total_cells = size * size
    num_obstacles = int(total_cells * obstacle_density)

    obstacles_placed = 0
    while obstacles_placed < num_obstacles:
        r = random.randint(0, size - 1)
        c = random.randint(0, size - 1)

        # Making sure that we don't place obstacles on the start or goal positions
        if (r, c) != start and (r, c) != goal and grid[r, c] == 0:
            grid[r, c] = 2
            obstacles_placed += 1

    return grid, start, goal