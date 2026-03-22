"""
Purpose of astar.py: It is central to path planning logic. It includes both the Standard A* algorithm,
which focuses solely on the shortest path, and the Penalty-based A* algorithm,
which considers the distance to obstacles.
"""
import heapq
import math

class Node:
    """
    A class representing each state (coordinate) and cost in the search space.
    """
    def __init__(self, x, y, g=0.0, h=0.0, parent=None):
        self.x = int(x)
        self.y = int(y)
        self.g = float(g)
        self.h = float(h)
        self.f = self.g + self.h
        self.parent = parent

    def __lt__(self, other):
        return self.f < other.f

def heuristic(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])

def astar(grid, start, goal):
    """
    Standard A* Algorithm. Aims to find the shortest path,
    disregarding safety distance when going around obstacles.
    """
    rows, cols = grid.shape
    open_list = []
    closed = set()

    start_node = Node(int(start[0]), int(start[1]), 0.0, heuristic(start, goal))
    heapq.heappush(open_list, start_node)

    while open_list:
        current = heapq.heappop(open_list)

        if (current.x, current.y) == goal:
            path = []
            while current:
                path.append((current.x, current.y))
                current = current.parent
            return path[::-1]

        closed.add((current.x, current.y))

        for dx, dy in [(-1,0),(1,0),(0,-1),(0,1),
                       (-1,-1),(-1,1),(1,-1),(1,1)]:

            nx = int(current.x + dx)
            ny = int(current.y + dy)

            if 0 <= nx < rows and 0 <= ny < cols:
                if grid[nx][ny] == 2:  # Obstacle check
                    continue
                if (nx, ny) in closed:
                    continue

                # Apply penalty for rough terrain (1)
                terrain_multiplier = 2.0 if grid[nx][ny] == 1 else 1.0
                g = current.g + (math.hypot(dx, dy) * terrain_multiplier)
                h = heuristic((nx, ny), goal)

                neighbor = Node(nx, ny, g, h, current)
                heapq.heappush(open_list, neighbor)

    return None

def astar_with_penalty(grid, start, goal, distance_map, lambda_weight=5):
    """
    Hybrid (Penalty-Induced) A* Algorithm.
    Adds an extra penalty to the standard distance cost (g+h) based on the cell's proximity to the obstacle.
    This allows the vehicle to prefer wider and safer curves rather than narrow spaces.
    """
    rows, cols = grid.shape
    open_list = []
    closed = set()

    start_node = Node(int(start[0]), int(start[1]), 0.0, heuristic(start, goal))
    heapq.heappush(open_list, start_node)

    while open_list:
        current = heapq.heappop(open_list)

        if (current.x, current.y) == goal:
            path = []
            while current:
                path.append((current.x, current.y))
                current = current.parent
            return path[::-1]

        closed.add((current.x, current.y))

        for dx, dy in [(-1,0),(1,0),(0,-1),(0,1),
                       (-1,-1),(-1,1),(1,-1),(1,1)]:

            nx = int(current.x + dx)
            ny = int(current.y + dy)

            if 0 <= nx < rows and 0 <= ny < cols:
                if grid[nx][ny] == 2:  # Obstacle check
                    continue
                if (nx, ny) in closed:
                    continue

                # Apply penalty for rough terrain (1)
                terrain_multiplier = 2.0 if grid[nx][ny] == 1 else 1.0
                g = current.g + (math.hypot(dx, dy) * terrain_multiplier)
                h = heuristic((nx, ny), goal)

                # Dynamic penalty based on proximity to nearest obstacle
                dist = distance_map[nx][ny]
                penalty = 1.0 / (dist + 1.0)
                f = g + h + (lambda_weight * penalty)

                neighbor = Node(nx, ny, g, h, current)
                neighbor.f = f

                heapq.heappush(open_list, neighbor)

    return None