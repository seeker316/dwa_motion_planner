### Dynamic Window Approach (DWA) for Local Path Planning

The **Dynamic Window Approach (DWA)** is a real-time, trajectory-based **local path planning algorithm** commonly used in mobile robotics. It’s widely used in robotics for **differential drive and holonomic robots**.

The DWA algorithm works in three main steps:
1. **Velocity Sampling**:  
    At each time step, the algorithm samples multiple combinations of **linear** and **angular velocities** within the robot's dynamic constraints (such as maximum speed)
2. **Trajectory Simulation**:  
    Each velocity sample is used to simulate the robot’s **short-term future trajectory**, predicting its path over a short time horizon.
3. **Trajectory Scoring and Selection**  
	Each trajectory is scored based on the following criteria like:
	- **Obstacle Avoidance**: Trajectories that risk collision are penalized.
	- **Goal Alignment**: Trajectories are scored higher if they head toward the goal.
	- **Speed Efficiency**: Faster and dynamically feasible trajectories are preferred.
	    The best trajectory is selected, and the corresponding velocity command is issued.

### Project Description

This project implements the DWA algorithm as a **ROS 2 Action Server**, allowing external nodes (like an action client) to send navigation goals. The planner computes and returns a safe local trajectory for the robot.
- **ROS 2 Version Recommended**: Humble or higher
- **Simulation Tools**: Gazebo, RViz2
- **Robot Model**: TurtleBot3
- **Dependencies**:  This package requires the `turtlebot3_gazebo` and `turtlebot3_bringup` packages.

### Installation Instructions

1. Install Required Packages :  Ensure you have the required TurtleBot3 simulation packages installed
```bash
sudo apt update
sudo apt install \
  ros-humble-turtlebot3-gazebo \
  ros-humble-turtlebot3-bringup
```
2. Clone and Build the Package
```bash
mkdir -p ~/dwa_ws/src
cd ~/dwa_ws/src

git clone <your-dwa-motion-planner-repo-url> dwa_motion_planner

cd ~/dwa_ws

rosdep install --from-paths src --ignore-src -r -y

colcon build --packages-select dwa_motion_planner

source install/setup.bash
```
3. Test and Run : To start the simulation and planning system
```bash
ros2 launch dwa_motion_planner dwa_launch.py
```
This launch file will:
- Start the **TurtleBot3 Gazebo simulation world**.
- Launch **RViz2** with the TurtleBot3 configuration.
- Start the **`dwa_action_server`**.
- Allow an external **`dwa_action_client`** to send navigation goals with `goal_x` and `goal_y` coordinates.

### Code Documentation
#### `dwa_action_server`
Implement a **Dynamic Window Approach (DWA)** based motion planner as a **ROS 2 Action Server** under the action name `"dwa_planner"`. This planner enables a robot to navigate toward a goal while avoiding obstacles, utilizing input from `LaserScan` and `Odometry` data.

**Workflow**:
- **Goal Received:** Extracts target `(x, y)` position and accepts the goal.    
- **Path Generation:** Randomly samples velocity and turn-rate combinations.
- **Scoring:**
    - Distance to goal
    - Heading alignment
    - Collision risk 
    - Smoothness
- **Best Path Selection:** Chooses and publishes the best-scored path.
- **Execution:** Publishes velocity commands until goal is reached or aborted.
- **Visualization:** Publishes top paths as RViz markers for debugging.

**Topics**:
- **Subscribed:**
    - `/odom` (`nav_msgs/msg/Odometry`): Robot's pose and velocity.
    - `/scan` (`sensor_msgs/msg/LaserScan`): Laser scan for obstacle data.
- **Published:**
    - `/cmd_vel` (`geometry_msgs/msg/TwistStamped`): Velocity commands to the robot.
    - `/visual_paths` (`visualization_msgs/msg/Marker`): Visual markers of evaluated paths.

**Actions**:
Type: `dwa_motion_planner/action/DwaPlanner`
- **Goal:**
    - `goal_x` (float)
    - `goal_y` (float)
- **Feedback:**
    - `distance_remaining` (float)
- **Result:**
    - `reached` (bool): True if goal reached, else false.

**Method Descriptions**:
- **`DWAPlanner()`** – Initializes the node, sets up action server, subscriptions, publishers, and RNG.
- **`handle_goal()`** – Accepts incoming action goal and extracts target coordinates.
- **`handle_cancel()`** – Handles client requests to cancel the current goal.
- **`handle_accepted()`** – Spawns a new thread to execute the accepted goal.
- **`odom_callback()`** – Stores latest odometry message for position and orientation.
- **`laserscan_callback()`** – Stores latest laser scan data for obstacle detection.
- **`publish_path_marker()`** – Publishes a visual marker of a predicted path in RViz.
- **`clear_all_markers()`** – Sends a message to delete all RViz path markers.
- **`predict_motion()`** – Simulates robot motion given speed and turn rate over a time horizon.
- **`generate_path()`** – Randomly generates a velocity and turn-rate pair for path sampling.
- **`check_for_collisions()`** – Checks if a predicted path collides with obstacles using laser data.
- **`choose_best_path()`** – Evaluates multiple random paths and returns the highest scoring one.
- **`execute()`** – Runs the main control loop for navigating toward the goal while publishing feedback.

#### `dwa_action_client`
Implements a ROS 2 Action Client node to send target goal positions to the DWA Action Server (`"dwa_planner"`). It allows initiating a motion planning request and monitors feedback and results from the server.

**Workflow**:
- **Parameter Declaration:** Reads goal coordinates (`goal_x`, `goal_y`) from node parameters (default: 2.0, 1.0).
- **Action Server Sync:** Waits for `"dwa_planner"` action server to be available.
- **Goal Dispatch:** Sends a `DwaPlanner` goal to the server.
- **Feedback Handling:** Displays the remaining distance to the goal in real-time.
- **Result Handling:** Reports whether the robot successfully reached the goal or failed.

**Method Descriptions**:
- **`DWAClient()`** – Initializes the action client, declares parameters, waits for the server, and sends the goal.
- **`feedback_callback()`** – Receives and logs distance remaining to the target in real-time.
- **`result_callback()`** – Logs success or failure of the goal once execution completes.
