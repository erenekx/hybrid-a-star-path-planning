import numpy as np
from collections import deque

def compute_distance_map(grid):
    rows, cols = grid.shape
    distance = np.full((rows, cols), np.inf)

    queue = deque()

    for i in range(rows):
        for j in range(cols):
            if grid[i][j] == 1:
                distance[i][j] = 0
                queue.append((i, j))

    directions = [(-1,0),(1,0),(0,-1),(0,1),
                  (-1,-1),(-1,1),(1,-1),(1,1)]

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