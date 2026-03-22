"""
Purpose of distance_map.py:
Computes the "Distance Map" for the grid using BFS.
Provides a time complexity of O(1) lookup table for A* to instantly find the distance
from any cell to the nearest obstacle.
"""
import numpy as np
from collections import deque

def compute_distance_map(grid):
    rows, cols = grid.shape
    distance = np.full((rows, cols), np.inf)

    queue = deque()

    # CRITICAL FIX: In grid_map.py, obstacles are marked as 2 (not 1).
    # We only calculate distance to actual impassable blocked cells.
    for i in range(rows):
        for j in range(cols):
            if grid[i][j] == 2:
                distance[i][j] = 0
                queue.append((i, j))

    directions = [(-1, 0), (1, 0), (0, -1), (0, 1),
                  (-1, -1), (-1, 1), [1, -1], (1, 1)]

    # BFS: Propagates distance waves outwards
    # from all obstacles simultaneously.
    while queue:
        x, y = queue.popleft()

        for dx, dy in directions:
            nx = x + dx
            ny = y + dy

            if 0 <= nx < rows and 0 <= ny < cols:
                new_dist = distance[x][y] + 1

                if new_dist < distance[nx][ny]:
                    distance[nx][ny] = new_dist
                    queue.append((nx, ny))

    return distance