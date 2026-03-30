import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
import numpy as np
import math
from driftbot_control.drift_mpc import DriftMPC

class DriftBotNode(Node):
    def __init__(self):
        super().__init__('driftbot_mpc_node')
        # ACTUATORS: Using cmd_vel for ESC Emulation
        self.steer_l_pub = self.create_publisher(Float64, '/model/driftbot/joint/front_left_steer_joint/cmd_pos', 10)
        self.steer_r_pub = self.create_publisher(Float64, '/model/driftbot/joint/front_right_steer_joint/cmd_pos', 10)
        self.vel_l_pub = self.create_publisher(Float64, '/model/driftbot/joint/rear_left_wheel_joint/cmd_vel', 10)
        self.vel_r_pub = self.create_publisher(Float64, '/model/driftbot/joint/rear_right_wheel_joint/cmd_vel', 10)
        
        self.odom_sub = self.create_subscription(Odometry, '/model/driftbot/odometry', self.odom_callback, 10)
        self.mpc = DriftMPC()
        self.current_state = np.zeros(6) 
        self.u0 = np.zeros((self.mpc.N, 2)); self.X0 = np.zeros((self.mpc.N + 1, 6))
        self.virtual_throttle = 1.0 # Gentle launch
        self.track_cx, self.track_cy, self.center_locked = 0.0, 0.0, False
        self.timer = self.create_timer(self.mpc.dt, self.control_loop)
        
    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        v_lin = msg.twist.twist.linear; v_ang = msg.twist.twist.angular
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        
        # TRANSFORMATION: Global Velocity to Local Chassis Speed (CRITICAL FOR DRIFT)
        vx = v_lin.x * math.cos(yaw) + v_lin.y * math.sin(yaw)
        vy = -v_lin.x * math.sin(yaw) + v_lin.y * math.cos(yaw)
        self.current_state = np.array([pos.x, pos.y, yaw, vx, vy, v_ang.z])
        
        if not self.center_locked:
            self.track_cx, self.track_cy, self.center_locked = pos.x, pos.y - 4.5, True

    def generate_drift_reference(self):
        P = np.zeros(6 + 6 * self.mpc.N); P[0:6] = self.current_state
        if not self.center_locked: return P 

        current_vx = self.current_state[3]
        track_radius, target_vx, turn_dir = 4.5, 6.0, -1.0 
        
        # TAKUMI DESIGN: Safety-First slip ramping
        # Ramps to 35-deg slip (0.6 rad) ONLY when the car exceeds 4.0 m/s
        drift_ratio = max(0.0, min(1.0, (current_vx - 4.0) / 1.5))
        beta = 0.6 * turn_dir * drift_ratio
        
        # THE FIX: Path points only move as fast as the chassis (max 1.5m/s buffer)
        path_vel = max(1.5, current_vx)
        dx, dy = self.current_state[0] - self.track_cx, self.current_state[1] - self.track_cy
        curr_angle = math.atan2(dy, dx)

        for k in range(self.mpc.N):
            future_a = curr_angle + ((path_vel / track_radius) * turn_dir * k * self.mpc.dt)
            idx = 6 + k * 6
            # [x, y, psi, vx, vy, r]
            P[idx:idx+6] = [self.track_cx + track_radius*math.cos(future_a), 
                            self.track_cy + track_radius*math.sin(future_a), 
                            (future_a - math.pi/2) + beta, target_vx, 0.0, (target_vx/track_radius)*turn_dir]
        return P

    def control_loop(self):
        if not self.center_locked: return
        P_target = self.generate_drift_reference()
        
        # Solve MPC using your existing CasADi brain
        sol = self.mpc.solver(x0=np.concatenate([self.X0.flatten(), self.u0.flatten()]), p=P_target, lbg=0, ubg=0, 
                               lbx=np.concatenate([np.full(6*(self.mpc.N+1), -np.inf), np.tile([-0.8, -5.0], self.mpc.N)]),
                               ubx=np.concatenate([np.full(6*(self.mpc.N+1), np.inf), np.tile([0.8, 5.0], self.mpc.N)]))
        
        opt_controls = sol['x'].full().flatten()[6*(self.mpc.N+1):].reshape((self.mpc.N, 2))
        
        # ESC Emulation: Accumulate target chassis speed
        self.virtual_throttle = max(2.0, min(8.0, self.virtual_throttle + opt_controls[0,1]*self.mpc.dt))
        
        # DRIFT KICK: Shatter static friction once momentum is established
        if self.current_state[3] > 4.5 and abs(self.current_state[4]) < 0.2 and not self.drift_kick_done:
            optimal_delta = 0.8 * np.sign(optimal_delta + 1e-5) 
            self.virtual_throttle = 15.0 
            self.drift_kick_done = True
            self.get_logger().info(">>> DRIFT INITIATED: MODULATING... <<<")
        else:
            # Let the MPC naturally control the throttle to maintain the slide
            self.virtual_throttle = max(3.0, min(9.0, self.virtual_throttle + opt_controls[0,1]*self.mpc.dt))


        wheel_omega = self.virtual_throttle / 0.0325
        # ... (Publish steer to cmd_pos and omega to cmd_vel) ...
        s_msg = Float64(data=float(opt_controls[0,0]))
        v_msg = Float64(data=float(wheel_omega))
        self.steer_l_pub.publish(s_msg); self.steer_r_pub.publish(s_msg)
        self.vel_l_pub.publish(v_msg); self.vel_r_pub.publish(v_msg)
        self.get_logger().info(f"vx: {self.current_state[3]:.1f} | vy: {self.current_state[4]:.2f}")

def main(args=None):
    rclpy.init(args=args); node = DriftBotNode(); rclpy.spin(node)

if __name__ == '__main__':
    main()