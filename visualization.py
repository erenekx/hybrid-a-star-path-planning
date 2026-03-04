import matplotlib.pyplot as plt

def visualize(grid, path, start, goal):

    plt.imshow(grid, cmap='gray_r')

    if path:
        xs = [p[1] for p in path]
        ys = [p[0] for p in path]
        plt.plot(xs, ys)

    plt.scatter(start[1], start[0])
    plt.scatter(goal[1], goal[0])

    plt.show()


def visualize_distance(distance_map):

    plt.imshow(distance_map, cmap='hot')
    plt.colorbar()
    plt.title("Obstacle Distance Map")
    plt.show()