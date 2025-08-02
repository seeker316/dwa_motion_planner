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
