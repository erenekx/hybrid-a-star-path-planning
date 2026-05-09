Hybrid A* Path Planning for Autonomous Vehicles

A Python-based autonomous vehicle path planning project implementing and extending the Hybrid A* algorithm in a 2D grid environment.

This project focuses on realistic autonomous navigation concepts such as:

* Non-holonomic vehicle constraints
* Obstacle avoidance
* Hybrid A* search
* Reeds-Shepp inspired motion behavior
* Dynamic obstacle simulation
* Vehicle animation and visualization
* Performance analysis and comparison

The project was developed incrementally following a 9-week academic roadmap.

⸻

Project Overview

Hybrid A* is one of the most widely used path planning algorithms in autonomous driving systems because it combines:

* Classical graph search
* Vehicle kinematics
* Realistic steering behavior

Unlike standard A*, Hybrid A* considers:

* Vehicle orientation
* Turning constraints
* Forward/reverse-like motion behavior
* Continuous movement approximation

This project demonstrates how autonomous vehicles can generate collision-free and realistic paths inside obstacle-filled environments.

⸻

Implemented Features

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

Visualization

* Vehicle movement animation
* Real-time path visualization
* Obstacle rendering
* Search exploration display

Analysis & Evaluation

* A* vs Hybrid A* comparison
* Execution time analysis
* Path length comparison
* Safety performance evaluation
* Heuristic sensitivity analysis

⸻

Weekly Roadmap Completion

Week 1 — Literature Review & Planning

Completed:

* Hybrid A* research
* Reeds-Shepp curve research
* Autonomous navigation study
* Project planning and presentation preparation

Week 2 — Basic A* Implementation

Completed:

* Grid map generation
* Basic A* implementation
* Obstacle handling
* Initial demo environment

Week 3 — Hybrid A* with Kinematics

Completed:

* Hybrid A* implementation
* Vehicle orientation support
* Kinematic movement constraints
* A* vs Hybrid A* comparison

Week 4 — Testing & Optimization

Completed:

* Obstacle-heavy environment testing
* Heuristic optimization
* Performance graph generation
* Safety penalty analysis

Week 5 — Visualization & Dynamic Obstacles

Completed:

* Vehicle animation system
* Dynamic obstacle simulation
* Real-time visualization
* Live simulation demo

Week 6 — Large Map Testing & Performance Analysis

Completed:

* Large-scale environment testing
* Stress testing
* Performance benchmarking
* Algorithm analysis

Note: CARLA simulator integration was not implemented in this version.
The project instead focuses on a custom Python-based simulation environment.

Week 7 — User Interface & Draft Report

Completed:

* Simulation interface improvements
* Visualization refinements
* Draft documentation preparation
* Full project demonstration

Week 8 — Final Experiments & Analysis

Completed:

* Final testing
* Complexity evaluation
* Optimality analysis
* Experimental comparisons
* Final report preparation

Week 9 — Final Presentation & Delivery

Completed:

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
├── utils.py
├── requirements.txt
├── README.md
│
├── assets/
│   ├── screenshots/
│   ├── animations/
│   └── graphs/
│
└── docs/
    ├── reports/
    └── presentations/

⸻

How the System Works

1. A grid-based environment is created.
2. Obstacles are placed inside the map.
3. Start and goal positions are defined.
4. The planner searches for a collision-free path.
5. Hybrid A* generates more realistic vehicle trajectories.
6. The final path is animated and analyzed.

⸻

Running the Project

Clone Repository

git clone https://github.com/your-username/hybrid-a-star-path-planning.git
cd hybrid-a-star-path-planning

Install Dependencies

pip install -r requirements.txt

Run Simulation

python main.py

⸻

Example Outputs

The project can generate:

* Autonomous vehicle animations
* Obstacle avoidance demonstrations
* Search visualization
* Performance graphs
* Hybrid A* trajectory comparisons

⸻

Educational Objectives

This project was developed to better understand:

* Autonomous vehicle navigation
* Search algorithms
* Heuristic optimization
* Vehicle motion planning
* Pathfinding under constraints
* Simulation and visualization systems

⸻

Limitations

Current limitations include:

* No real sensor integration
* No CARLA simulator integration
* 2D environment only
* Simplified vehicle dynamics

Despite these limitations, the project successfully demonstrates the core principles of Hybrid A* based autonomous path planning.

⸻

Future Improvements

Potential future work:

* CARLA integration
* ROS support
* LiDAR simulation
* Real vehicle kinematics
* Continuous-space planning
* Multi-agent navigation

⸻

Contributors

* Eren
* Project Team Members

⸻

License

This project was developed for educational and academic purposes.
