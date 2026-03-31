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

# Week 4 – Optimization and Performance Improvements

In Week 4, the focus was on improving the efficiency and structure of the A* algorithm.

### Improvements
- Optimized the open list using a priority queue (heap)
- Reduced unnecessary node expansions
- Improved cost calculations for better performance
- Refactored code for readability and maintainability

### Results
- Faster path computation
- More stable algorithm behavior
- Cleaner and modular code structure

---

# Week 5 – Visualization, Animation and Dynamic Replanning

In Week 5, the project was extended with visualization and dynamic behavior to simulate real-world scenarios.

### Features
- Vehicle animation on the grid
- Step-by-step path traversal
- Path smoothing for more natural movement
- Turn-aware speed adjustment

### Dynamic Obstacle System
- Introduced obstacles that appear on the planned path
- Simulated real-time changes in the environment

### Real-Time Replanning
- When the obstacle blocks the path:
  - The algorithm recomputes a new path
  - The vehicle avoids the obstacle
  - A new trajectory is followed dynamically

### Outcome
- Demonstrated intelligent navigation behavior
- Enabled real-time decision making
- Improved visualization for clearer understanding

---

# Project Files

**main.py**  
Main execution script for running experiments and simulations.

**astar.py**  
Contains the standard A* implementation and the penalty-based A* algorithm.

**distance_map.py**  
Computes the distance from each grid cell to the nearest obstacle.

**grid_map.py**  
Creates the test grid environment.

**visualization.py**  
Handles visualization, animation, and dynamic obstacle simulation.