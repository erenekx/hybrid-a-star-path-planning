import matplotlib
matplotlib.use("TkAgg")

import matplotlib.pyplot as plt
import numpy as np
import time


def visualize(grid, path=None, start=None, goal=None, title="Path Visualization"):
    grid = np.array(grid)

    plt.figure()
    plt.imshow(grid, cmap='gray_r')

    if path:
        xs = [p[1] for p in path]
        ys = [p[0] for p in path]
        plt.plot(xs, ys, linewidth=2)

    if start:
        plt.plot(start[1], start[0], "go")

    if goal:
        plt.plot(goal[1], goal[0], "ro")

    plt.title(title)
    plt.show()


def animate_path(grid, path, start, goal):
    import copy
    from astar import astar_with_penalty

    fig, ax = plt.subplots()
    base_grid = np.array(grid)

    current = start

    trail_x = []
    trail_y = []

    # 🔥 başlangıç obstacle konumu (path üstünde)
    obstacle_index = len(path) // 3
    direction = 1
    step_counter = 0

    while current != goal:

        step_counter += 1

        # 🔥 obstacle yavaş hareket (her 3 adımda 1)
        if step_counter % 3 == 0:
            obstacle_index += direction

            # çok uzaklaşmasın → hep karşılaşma bölgesinde kalsın
            if obstacle_index > len(path)//2 or obstacle_index < len(path)//4:
                direction *= -1

        ox, oy = path[obstacle_index]

        temp_grid = copy.deepcopy(base_grid)
        temp_grid[int(ox), int(oy)] = 1

        # 🔥 replanning
        new_path = astar_with_penalty(
            temp_grid,
            current,
            goal,
            None,
            lambda_weight=50
        )

        if not new_path or len(new_path) < 2:
            print("Path blocked!")
            break

        next_step = new_path[1]
        current = next_step

        x, y = current

        trail_x.append(y)
        trail_y.append(x)

        # --- DRAW ---
        ax.clear()
        ax.imshow(temp_grid, cmap='gray_r')

        ax.plot(trail_x, trail_y, linewidth=2)
        ax.plot(y, x, "ks", markersize=6)

        # 🔴 dynamic obstacle
        ax.plot(oy, ox, "ro", markersize=9)

        ax.plot(start[1], start[0], "go")
        ax.plot(goal[1], goal[0], "ro")

        ax.set_xlim(-1, base_grid.shape[1])
        ax.set_ylim(base_grid.shape[0], -1)

        plt.pause(0.05)
        time.sleep(0.02)

    plt.show()