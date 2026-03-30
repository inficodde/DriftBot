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

**1. Source your ROS 2 environment:**
`source /opt/ros/humble/setup.bash`

**2. Build the Workspace:**
`colcon build --packages-select driftbot_control && source install/setup.bash`

**3. Launch the Simulation:**
`ign gazebo -r ~/drift_bot_ws/src/drift_world.sdf`

**4. Start the Communication Bridge:**
```bash
ros2 run ros_gz_bridge parameter_bridge /model/driftbot/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry /model/driftbot/joint/front_left_steer_joint/cmd_pos@std_msgs/msg/Float64@ignition.msgs.Double /model/driftbot/joint/front_right_steer_joint/cmd_pos@std_msgs/msg/Float64@ignition.msgs.Double /model/driftbot/joint/rear_left_wheel_joint/cmd_vel@std_msgs/msg/Float64@ignition.msgs.Double /model/driftbot/joint/rear_right_wheel_joint/cmd_vel@std_msgs/msg/Float64@ignition.msgs.Double


## 🛠️ Troubleshooting & Common Fixes

### 🛸 The "Flying Audi" (Physics Instability)
* **Issue:** The car somersaults or explodes into the air upon spawning.
* **Cause:** Usually a "Divide by Zero" in the physics engine due to missing `<inertial>` tags in the wheels or the chassis collision box overlapping with the wheel cylinders.
* **Fix:** Ensure all links in `model.sdf` have mass/inertia and the chassis box width is restricted to `0.18m` to clear the wheels.

### 🧊 The Car Won't Move (Actuation)
* **Issue:** Logs show the MPC is working, but the car is stationary in Gazebo.
* **Cause 1:** Missing `p_gain` in the `JointController` plugins.
* **Cause 2:** Topic mismatch between the Node (`cmd_vel`) and the Bridge.
* **Fix:** Verify `p_gain` is set to at least `10.0` in `model.sdf` and check `ign topic -l` to ensure the topics match the bridge.

### 👻 The "Binary Ghost" (Old Code Running)
* **Issue:** You edited the Python code, but the terminal logs still show old variables (e.g., logging "Torque" instead of "vx").
* **Cause:** ROS 2 is running a stale binary from the `install/` folder.
* **Fix:** Run `rm -rf build/ install/ log/` and rebuild with `colcon build`.

### 🏠 Missing Models
* **Issue:** Gazebo fails to load the Audi R8 mesh or the Prius hybrid textures.
* **Fix:** Ensure your resource path is exported in your terminal:
  `export IGN_GAZEBO_RESOURCE_PATH=~/drift_bot_ws/src/models`

### 🧱 Octagon Wall-Slamming
* **Issue:** The car drifts perfectly in the open lot but hits the inner wall of the octagon.
* **Cause:** The "Ghost Rabbit" reference is projecting too far ahead, causing the MPC to "cut the corner."
* **Fix:** Decrease the projection look-ahead in `generate_drift_reference` or increase the $Q$ matrix weight for $X, Y$ in `drift_mpc.py`.
