import numpy as np

def create_test_map():
    grid = np.zeros((20, 20))

    # obstacles
    grid[5:15, 10] = 1
    grid[10, 5:15] = 1

    start = (2, 2)
    goal = (17, 17)

    return grid, start, goal