Hybrid A* Path Planning for Autonomous Vehicles

A comprehensive autonomous vehicle path planning project implementing and extending the Hybrid A* algorithm in both custom 2D environments and the CARLA simulator.

This project focuses on realistic autonomous navigation concepts such as obstacle avoidance, vehicle kinematics, smooth trajectory generation, dynamic obstacle handling, and autonomous driving simulation.

⸻

Project Overview

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

Features

Path Planning Algorithms

* Basic A* Algorithm
* Safety-Aware A*
* Hybrid A* Path Planning
* Obstacle-aware heuristic system

Vehicle & Motion Simulation

* Orientation-aware movement
* Turning radius constraints
* Reeds-Shepp inspired motion behavior
* Collision checking

Environment System

* 2D grid-based maps
* Static obstacle support
* Dynamic obstacle simulation
* Safe path generation
* CARLA simulator testing

Visualization

* Vehicle movement animation
* Real-time path visualization
* Obstacle rendering
* Search exploration display
* CARLA environment demonstrations

Analysis & Evaluation

* A* vs Hybrid A* comparison
* Execution time analysis
* Path length comparison
* Safety performance evaluation
* Heuristic sensitivity analysis
* Experimental benchmarking

⸻

Path Planning Algorithm Comparison

Algorithm	Type	Advantages	Disadvantages
A*	Graph-based	Optimal and fast	No kinematic constraints
Hybrid A*	Graph + Kinematic	Kinematically feasible and smooth paths	More complex and slower
RRT	Sampling-based	Effective in high-dimensional spaces	Produces sub-optimal paths
RRT*	Sampling-based	Asymptotically optimal	Slow convergence
PRM	Sampling-based	Good for multiple queries	Requires preprocessing
Lattice Planner	Graph-based	Predefined feasible motions	Memory intensive
D* / D* Lite	Incremental Search	Suitable for dynamic environments	No kinematic constraints
Dijkstra	Graph-based	No heuristic required	Slower than A*
Potential Field	Force-based	Real-time capable	Local minima problem
MPC	Optimization-based	Handles dynamic constraints	High computational cost

⸻

Why Hybrid A*?

This project does not claim that Hybrid A* is universally the best path planning algorithm.

Instead, the project demonstrates that Hybrid A* performs very effectively for autonomous driving scenarios where:

* Vehicle kinematics matter
* Smooth trajectories are required
* Obstacle avoidance is critical
* Realistic driving behavior is necessary

The algorithm was evaluated through comparative experimental analysis using metrics such as:

* Path smoothness
* Collision avoidance capability
* Execution time
* Path optimality
* Realistic vehicle movement
* Dynamic environment performance

⸻

Weekly Roadmap Completion

Week 1 — Literature Review & Planning

* Hybrid A* research
* Reeds-Shepp curve research
* Autonomous navigation study
* Project planning and presentation preparation

Week 2 — Basic A* Implementation

* Grid map generation
* Basic A* implementation
* Obstacle handling
* Initial demo environment

Week 3 — Hybrid A* with Kinematics

* Hybrid A* implementation
* Vehicle orientation support
* Kinematic movement constraints
* A* vs Hybrid A* comparison

Week 4 — Testing & Optimization

* Obstacle-heavy environment testing
* Heuristic optimization
* Performance graph generation
* Safety penalty analysis

Week 5 — Visualization & Dynamic Obstacles

* Vehicle animation system
* Dynamic obstacle simulation
* Real-time visualization
* Live simulation demo

Week 6 — CARLA Integration & Large Map Testing

* CARLA simulator integration
* 3D autonomous driving environment testing
* Large-scale map experiments
* Performance benchmarking
* Navigation analysis in CARLA

Week 7 — User Interface & Draft Report

* Simulation interface improvements
* Visualization refinements
* Draft documentation preparation
* Full project demonstration

Week 8 — Final Experiments & Analysis

* Final testing
* Complexity evaluation
* Optimality analysis
* Experimental comparisons
* Final report preparation

Week 9 — Final Presentation & Delivery

* Final presentation preparation
* Video/demo preparation
* Code cleanup and organization
* Project delivery

⸻

Technologies Used

* Python
* NumPy
* Matplotlib
* Heapq
* CARLA Simulator
* Object-Oriented Programming

⸻

Project Structure

hybrid-a-star-path-planning/
│
├── main.py
├── astar.py
├── hybrid_astar.py
├── environment.py
├── visualization.py
├── carla_simulation.py
├── utils.py
├── requirements.txt
├── README.md
│
├── assets/
│   ├── screenshots/
│   ├── animations/
│   ├── graphs/
│   └── carla_demo/
│
└── docs/
    ├── reports/
    └── presentations/

⸻

Installation

Clone Repository

git clone https://github.com/your-username/hybrid-a-star-path-planning.git
cd hybrid-a-star-path-planning

Install Dependencies

pip install -r requirements.txt

⸻

Usage

Run Main Simulation

python main.py

Run CARLA Simulation

python carla_simulation.py

⸻

Example Outputs

The project can generate:

* Autonomous vehicle animations
* Obstacle avoidance demonstrations
* Search visualization
* Performance graphs
* Hybrid A* trajectory comparisons
* CARLA autonomous driving demonstrations

⸻

Educational Objectives

This project was developed to better understand:

* Autonomous vehicle navigation
* Search algorithms
* Heuristic optimization
* Vehicle motion planning
* Pathfinding under constraints
* Simulation and visualization systems
* Realistic autonomous driving environments

⸻

Future Improvements

Potential future work:

* Real-time sensor fusion
* LiDAR integration
* ROS support
* Advanced vehicle dynamics
* Continuous-space planning
* Multi-agent navigation
* Machine learning assisted planning


⸻

License

This project was developed for educational and research purposes.
