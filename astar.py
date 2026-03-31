import heapq
import math


def heuristic(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def get_neighbors(node, grid):
    x, y = node

    directions = [
        (1, 0), (-1, 0),
        (0, 1), (0, -1)
    ]

    neighbors = []

    for dx, dy in directions:
        nx, ny = x + dx, y + dy

        if 0 <= nx < len(grid) and 0 <= ny < len(grid[0]):
            if grid[nx][ny] == 0:
                neighbors.append((nx, ny))

    return neighbors


def astar(grid, start, goal, distance_map=None, lambda_penalty=0.0, turn_penalty_weight=0.0):
    open_heap = []
    heapq.heappush(open_heap, (0, start))

    came_from = {}
    g_cost = {start: 0}
    closed = set()

    while open_heap:
        _, current = heapq.heappop(open_heap)

        if current in closed:
            continue

        if current == goal:
            break

        closed.add(current)

        for neighbor in get_neighbors(current, grid):

            if neighbor in closed:
                continue

            new_g = g_cost[current] + 1

            obstacle_penalty = 0
            if distance_map is not None:
                dist = distance_map[neighbor[0]][neighbor[1]]
                if dist > 0:
                    obstacle_penalty = 1.0 / (dist + 0.1)

            new_g_total = new_g + lambda_penalty * obstacle_penalty

            if neighbor not in g_cost or new_g_total < g_cost[neighbor]:
                g_cost[neighbor] = new_g_total

                f = new_g_total + heuristic(neighbor, goal)

                heapq.heappush(open_heap, (f, neighbor))
                came_from[neighbor] = current

    path = []
    node = goal

    if node not in came_from:
        return []

    while node != start:
        path.append(node)
        node = came_from[node]

    path.append(start)
    path.reverse()

    return path


def astar_with_penalty(grid, start, goal, distance_map, lambda_weight=1.0):
    return astar(
        grid,
        start,
        goal,
        distance_map=distance_map,
        lambda_penalty=lambda_weight,
        turn_penalty_weight=0.0
    )