"""
Purpose of grid_map.py:
The goal is to create a virtual environment/map for
testing simulations and algorithms.
"""
import numpy as np

def create_test_map():
    """
    It creates a sample map with dimensions of 20x20 and obstacles
    in the shape of a plus sign (cross) in the middle (1).
    """
    grid = np.zeros((20, 20))

    # Replacing obstacles as "plus(+)" visual
    grid[5:15, 10] = 1 # X-axis
    grid[10, 5:15] = 1 # Y-axis

    start = (2, 2)
    goal = (17, 17)

    return grid, start, goal