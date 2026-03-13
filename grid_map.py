"""
Purpose of grid_map.py:
Creates a virtual map environment for testing path planning algorithms.
"""

import numpy as np


def create_test_map():

    grid = np.zeros((20, 20))

    grid[5:15, 10] = 1
    grid[10, 5:15] = 1

    # Rough terrain (1)
    grid[3:8, 3:8] = 1
    grid[12:17, 12:17] = 1

    # Start and Goal
    start = (2, 2)
    goal = (17, 17)

    return grid, start, goal