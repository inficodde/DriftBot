# Driftbot 🏎️💨

An autonomous, 1/10th scale Rear-Wheel Drive (RWD) vehicle simulation built to explore modern control theory, vehicle dynamics, and high-slip angle navigation. 

## 🛠️ Tech Stack
* **Simulation:** Ignition Gazebo
* **Framework:** ROS 2
* **Telemetry & Visualization:** Foxglove Studio (Local)
* **Control Stack:** Python / `rclpy` (Transitioning to C++ / `rclcpp`)

## 🚀 Current Features
* Custom RWD chassis modeled with accurate physical mass (1.8kg) and inertia matrices.
* Custom local test track environment (`drift_world.sdf`).
* Bidirectional `ros_gz_bridge` pipeline passing IMU, 2D Lidar, Odometry, and `/tf` coordinate frames.
* Baseline reactive Autonomous Obstacle Avoidance (AOA) utilizing Lidar point clouds.

## 🏁 Next Steps (The Roadmap)
1. **Model Predictive Control (MPC):** Transitioning from reactive AOA to a Nonlinear MPC based on the Dynamic Bicycle Model to calculate and hold intentional slip angles (drifting).
2. **SLAM:** Integrating the ROS 2 SLAM Toolbox to map track boundaries and generate ideal racing lines.
3. **Computer Vision & AI:** Adding a camera for visual context and eventually applying Reinforcement Learning as an optimization layer over the MPC.

## 💻 How to Run Locally
**1. Source your ROS 2 environment:**
`source /opt/ros/humble/setup.bash`

**2. Set Gazebo to offline/local mode (prevents hanging):**
`export IGN_GAZEBO_RESOURCE_PATH=~/drift_bot_ws/src/models:~/.ignition/gazebo/models`

**3. Launch the Simulation:**
`ign gazebo -r ~/drift_bot_ws/src/drift_world.sdf`
