import heapq
import math

class Node:
    def __init__(self, x, y, g=0, h=0, parent=None):
        self.x = x
        self.y = y
        self.g = g
        self.h = h
        self.f = g + h
        self.parent = parent

    def __lt__(self, other):
        return self.f < other.f


def heuristic(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def astar(grid, start, goal):
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

                neighbor = Node(nx, ny, g, h, current)
                heapq.heappush(open_list, neighbor)

    return None


def astar_with_penalty(grid, start, goal, distance_map, lambda_weight=5):

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

                dist = distance_map[nx][ny]
                penalty = 1 / (dist + 1)

                f = g + h + lambda_weight * penalty

                neighbor = Node(nx, ny, g, h, current)
                neighbor.f = f

                heapq.heappush(open_list, neighbor)

    return None