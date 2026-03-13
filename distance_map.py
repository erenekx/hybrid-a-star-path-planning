"""
Purpose of distance_map.py:
It is used to calculate the distance of each empty (movable)
cell on the map to the nearest obstacle (cells with a value of 1).
"""
import numpy as np
from collections import deque

def compute_distance_map(grid):
    """
    Using a multi-initial Width-First Search (BFS) algorithm,
    it calculates the distance of each cell in the grid to the nearest obstacle.
    """
    rows, cols = grid.shape
    distance = np.full((rows, cols), np.inf)

    queue = deque()

    #Add all obstacles to the queue as starting points. (distance 0).
    for i in range(rows):
        for j in range(cols):
            if grid[i][j] == 1:
                distance[i][j] = 0
                queue.append((i, j))

    directions = [(-1,0),(1,0),(0,-1),(0,1),
                  (-1,-1),(-1,1),(1,-1),(1,1)]

    #By spreading in straight waves due to all the obstructions,
    #their distances are determined.
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