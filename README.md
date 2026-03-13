# Hybrid A* Path Planning Project

This project implements and analyzes path planning algorithms for autonomous vehicles in a grid-based environment.

---

# Week 1 – Project Setup and Environment

In the first stage of the project, the development environment and basic project structure were created.

### Objectives
- Define the project architecture
- Prepare the grid-based simulation environment
- Organize project modules for path planning experiments

### Implementations
- Created the project structure
- Implemented the grid environment generator
- Defined start and goal positions
- Prepared visualization utilities for later stages

---

# Week 2 – Baseline A* Path Planning

In this stage, the **standard A\*** path planning algorithm was implemented and tested on the grid environment.

### Features
- Baseline A* path planning
- Euclidean distance heuristic
- 8-directional grid movement
- Distance map computation for obstacle proximity

### Goal
The goal of this stage was to implement a working A* path planner that can compute the shortest path between the start and goal positions.

---

# Week 3 – Terrain Cost and Obstacle-Aware Planning

In Week 3, the algorithm was extended to consider **terrain cost and obstacle proximity** to generate safer paths.

### Improvements
- Added terrain types in the grid map
  - Normal terrain
  - Rough terrain
  - Obstacles
- Implemented a **penalty-based A\*** algorithm
- Introduced a **lambda (λ) parameter** to control obstacle avoidance behavior
- Compared standard A* and penalty-based A*

### Experiments

We evaluate the effect of the penalty parameter **λ** on:

- Path length
- Minimum obstacle distance
- Computation time

---

# Project Files

**main.py**  
Main execution script for running experiments and generating plots.

**astar.py**  
Contains the standard A* implementation and the penalty-based A* algorithm.

**distance_map.py**  
Computes the distance from each grid cell to the nearest obstacle.

**grid_map.py**  
Creates the test grid environment.

**visualization.py**  
Handles visualization of paths and distance maps.
