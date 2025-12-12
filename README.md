This project controls a Webots TurtleBot3 robot to build a 2D occupancy map using RBPF SLAM, 
then navigate to pickup/dropoff goals (or explore frontiers when no goal is available) using A* path planning and waypoint following.

How to run:
1) Open your Webots world.
2) Set the robot controller to epuck_main_controller.py.
3) Ensure all .py files are in the same controller directory so imports work.
4) Run the simulation.

Startup Behavior: 
* The robot spins for 5 seconds initially to gather scans, and set a goal.
* Then it begins goal persuit or exploration persuit.

What it does at each simulation step:
1)Reads wheel encoders and LiDAR
2) Updates odometry (odom.py)
3) Runs RBPF SLAM to estimate pose and update an occupancy grid map (rbpf_slam.py, slam.py)
4) Selects the next task goal (pickup/dropoff) or an exploration target (goals.py, go_to_goal.py, decision_layer.py)
5) Plans a path with A* and follows it using a lookahead controller (go_to_goal.py)
6) Applies reactive obstacle avoidance override (obstacle_avoidance.py)
7) Renders the map in Webots and/or via Matplotlib (print_map.py, map_debug.py)

What you will see running this: 
* occupancy grid drawn each second via print_map.py where:
  * white = free
  * grey = unknown
  * black = occupied
* Matplotlib debug plot: map_debug.py can overlay paths and goal points optionally.

  
