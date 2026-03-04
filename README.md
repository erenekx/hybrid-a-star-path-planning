# Hybrid A* Path Planning Project

This project implements and analyzes path planning algorithms for autonomous vehicles.

## Week 2
Implemented a grid-based A* planner with an obstacle-aware heuristic.

Features:
- Baseline A* path planning
- Obstacle distance map
- Safety-aware penalty heuristic
- Lambda parameter sensitivity analysis
- Visualization of paths and performance graphs

## Files

main.py
Main execution script for running experiments and generating plots.

astar.py
Contains the A* implementation and penalty-based heuristic version.

distance_map.py
Computes the distance from each grid cell to the nearest obstacle.

grid_map.py
Creates the test grid environment.

visualization.py
Handles visualization of paths and distance maps.

## Experiments

We evaluate the effect of the penalty parameter λ on:

- Path length
- Minimum obstacle distance
- Computation time
