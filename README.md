# Hybrid A* Path Planning for Autonomous Vehicles

A comprehensive autonomous vehicle path planning project implementing and extending the Hybrid A* algorithm in both custom 2D environments and the CARLA simulator.

This project focuses on realistic autonomous navigation concepts such as obstacle avoidance, vehicle kinematics, smooth trajectory generation, dynamic obstacle handling, and autonomous driving simulation.

⸻

## Project Overview

Path planning is one of the most critical components of autonomous driving systems. Different algorithms provide different advantages depending on the environment, motion constraints, and computational requirements.

This project focuses on Hybrid A* because it combines:
* Classical graph search
* Vehicle kinematics
* Smooth path generation
* Realistic steering behavior

Unlike traditional A*, Hybrid A* considers:
* Vehicle orientation
* Turning radius constraints
* Forward/reverse motion behavior
* Continuous vehicle movement approximation

The system demonstrates realistic autonomous navigation in obstacle-filled environments using both custom simulations and CARLA-based testing.

⸻

## Features

### Path Planning Algorithms
* **Basic A\* Algorithm**
* **Safety-Aware A\***
* **Hybrid A\* Path Planning**
* **Obstacle-aware heuristic system**

### Vehicle & Motion Simulation
* Orientation-aware movement
* Turning radius constraints (Kinematic Bicycle Model)
* Reeds-Shepp inspired motion behavior
* Footprint-based collision checking

### Environment System
* 2D grid-based maps
* Static obstacle support
* Dynamic obstacle simulation
* CARLA simulator testing

### Visualization
* Vehicle movement animation
* Real-time path visualization
* Obstacle rendering
* CARLA environment demonstrations

⸻

## Path Planning Algorithm Comparison

| Algorithm | Type | Advantages | Disadvantages |
| :--- | :--- | :--- | :--- |
| **A\*** | Graph-based | Optimal and fast | No kinematic constraints |
| **Hybrid A\*** | Graph + Kinematic | Feasible and smooth paths | More complex and slower |
| **RRT** | Sampling-based | Effective in high-dimensional spaces | Produces sub-optimal paths |
| **MPC** | Optimization-based | Handles dynamic constraints | High computational cost |
| **Dijkstra** | Graph-based | No heuristic required | Slower than A* |

⸻

## Why Hybrid A*?

This project demonstrates that Hybrid A* performs very effectively for autonomous driving scenarios where:
* Vehicle kinematics matter
* Smooth trajectories are required
* Obstacle avoidance is critical
* Realistic driving behavior is necessary

The algorithm was evaluated through comparative experimental analysis using metrics such as path smoothness, collision avoidance capability, and execution time.

⸻

## Weekly Roadmap Completion

* **Week 1 — Literature Review & Planning:** Hybrid A* research and project environment setup.
* **Week 2 — Basic A\* Implementation:** Grid map generation and basic A* implementation.
* **Week 3 — Terrain Cost & Obstacle-Aware Planning:** Penalty-based A* and terrain handling.
* **Week 4 — Hybrid A\* with Kinematics:** Non-holonomic constraints and Bicycle Model integration.
* **Week 5 — Visualization & Dynamic Obstacles:** Animation system and dynamic simulation.
* **Week 6 — CARLA Integration & Large Map Testing:** 3D environment experiments and benchmarking.
* **Week 7 — User Interface & Draft Report:** Simulation interface improvements and documentation.
* **Week 8 — Final Experiments & Analysis:** Complexity evaluation and optimality analysis.
* **Week 9 — Final Presentation & Delivery:** Code cleanup and project delivery.

⸻

## Technologies Used
* Python
* NumPy
* Matplotlib
* Heapq
* CARLA Simulator

⸻

## Project Structure
```text
hybrid-a-star-path-planning/
├── main.py
├── astar.py
├── hybrid_astar.py
├── environment.py
├── visualization.py
├── carla_simulation.py
├── utils.py
├── requirements.txt
└── README.md