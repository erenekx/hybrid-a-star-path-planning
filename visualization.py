""" Purpose of Visualization.py: This module is responsible for visualizing the maps,
routes created, and obstacle distance maps in the project."""
import matplotlib.pyplot as plt

def visualize(grid, path, start, goal, title="Path Planning"):
    """
    The grid plots the map,
    showing the start/end points and the calculated path.
    """
    plt.imshow(grid, cmap='gray_r')

    if path:
        xs = [p[1] for p in path]
        ys = [p[0] for p in path]
        plt.plot(xs, ys)

    plt.scatter(start[1], start[0], label="Start", color="green")
    plt.scatter(goal[1], goal[0], label="Final Destination", color="red")

    # Creating Graph with all details.
    plt.title(title)
    plt.legend()
    plt.show()


def visualize_distance(distance_map):
    """
    It plots the distance of each cell from the nearest obstacle as a heatmap.
    """
    plt.imshow(distance_map, cmap='hot')
    plt.colorbar()
    plt.title("Obstacle Distance Map")
    plt.show()