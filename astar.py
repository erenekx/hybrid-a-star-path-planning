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
    def __init__(self, x, y, g=0, h=0, parent=None):
        self.x = x
        self.y = y
        self.g = g  # Current cost from the beginning to this point.
        self.h = h  # The estimated (heuristic (in TR): tahmini) cost from this node to the target.
        self.f = g + h # Total cost score
        self.parent = parent

    def __lt__(self, other):
        # A comparison operator is used to enable the Priority Queue to select.
        # the lowest-cost element.
        return self.f < other.f

def heuristic(a, b):
    """Calculates the "Euclidean distance" as the crow flies between two points."""
    return math.hypot(a[0] - b[0], a[1] - b[1])

def astar(grid, start, goal):
    """
    Standard A* Algorithm. Aims to find the shortest path,
    disregarding safety distance when going around obstacles.
    """
    rows, cols = grid.shape
    open_list = []
    closed = set()

    start_node = Node(start[0], start[1], 0, heuristic(start, goal))
    heapq.heappush(open_list, start_node)

    while open_list:
        current = heapq.heappop(open_list)

        # If the destination has been reached,
        # trace the route back to determine the destination.
        if (current.x, current.y) == goal:
            path = []
            while current:
                path.append((current.x, current.y))
                current = current.parent
            return path[::-1]

        closed.add((current.x, current.y))

        for dx, dy in [(-1,0),(1,0),(0,-1),(0,1),
                       (-1,-1),(-1,1),(1,-1),(1,1)]:

            nx = current.x + dx
            ny = current.y + dy

            if 0 <= nx < rows and 0 <= ny < cols:
                if grid[nx][ny] == 1: # Obstacle Control.
                    continue
                if (nx, ny) in closed:
                    continue

                g = current.g + math.hypot(dx, dy)
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

    start_node = Node(start[0], start[1], 0, heuristic(start, goal))
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

            nx = current.x + dx
            ny = current.y + dy

            if 0 <= nx < rows and 0 <= ny < cols:
                if grid[nx][ny] == 1:
                    continue
                if (nx, ny) in closed:
                    continue

                g = current.g + math.hypot(dx, dy)
                h = heuristic((nx, ny), goal)

                # Calculation of obstacle penalty (Penalty increases as distance decreases).
                dist = distance_map[nx][ny]
                penalty = 1 / (dist + 1)
                f = g + h + lambda_weight * penalty # Lambda weighting was added to the total cost.

                neighbor = Node(nx, ny, g, h, current)
                neighbor.f = f

                heapq.heappush(open_list, neighbor)

    return None