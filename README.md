# Driftbot 🏎️💨

An autonomous, 1/10th scale Rear-Wheel Drive (RWD) vehicle simulation built to explore modern control theory, vehicle dynamics, and high-slip angle navigation. This project emulates the **TRI Takumi** architecture to achieve sustained, high-precision drifting at the limit of handling.

## 🛠️ Tech Stack
* **Simulation:** Ignition Gazebo
* **Framework:** ROS 2 Humble
* **Control Stack:** Python / `rclpy` (Using CasADi for Nonlinear MPC)
* **Physics Engine:** Bullet/DART (Tuned for low-friction RWD dynamics)

## 🏎️ Current Features
* **Nonlinear MPC (NMPC):** High-frequency controller utilizing a **Dynamic Bicycle Model** to calculate and hold intentional slip angles.
* **Audi R8 Physics Body:** Custom chassis model ($4.5\text{kg}$) with accurate inertia tensors and staggered friction coefficients ($\mu=0.25$ rear / $\mu=1.0$ front).
* **Velocity-Controlled Actuation:** Emulated ESC through `JointController` velocity commands (`cmd_vel`) to maintain traction and prevent simulator burnout.
* **Dynamic Reference Planning:** Velocity-anchored trajectory projection to prevent "Ghost Rabbit" runaway and wall collisions.

## 🏁 Next Steps (The Roadmap)
1. **Launch Control Tuning:** Refining $Q$ matrix weights to stabilize lane-keeping during high-torque initiation.
2. **SLAM & Mapping:** Integrating `slam_toolbox` to map track boundaries and optimize racing lines.
3. **Hardware Deployment:** Transitioning the stack to a physical Yokomo RWD drift chassis via micro-ROS.

## 💻 How to Run Locally

**1. Installation & Dependencies:**
To ensure the MPC solver and Gazebo plugins function correctly, install the following:

`
cd ~/drift_bot_ws
rosdep update
rosdep install --from-paths src --ignore-src -y`

`pip install -r src/driftbot_control/requirements.txt`

Note for Jazzy Users: > If you are on ROS 2 Jazzy, ensure you have the rebranded Gazebo Sim packages installed:

`sudo apt install ros-jazzy-ros-gz`

**2. Source your ROS 2 environment:**
`source /opt/ros/humble/setup.bash`

**3. Build the Workspace:**
`colcon build --packages-select driftbot_control && source install/setup.bash`

**4. Launch the Simulation:**
`ign gazebo -r ~/drift_bot_ws/src/drift_world.sdf`

**5. Start the Communication Bridge:**
```bash
ros2 run ros_gz_bridge parameter_bridge /model/driftbot/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry /model/driftbot/joint/front_left_steer_joint/cmd_pos@std_msgs/msg/Float64@ignition.msgs.Double /model/driftbot/joint/front_right_steer_joint/cmd_pos@std_msgs/msg/Float64@ignition.msgs.Double /model/driftbot/joint/rear_left_wheel_joint/cmd_vel@std_msgs/msg/Float64@ignition.msgs.Double /model/driftbot/joint/rear_right_wheel_joint/cmd_vel@std_msgs/msg/Float64@ignition.msgs.Double



