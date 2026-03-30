#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math

class AutonomousAOA(Node):
    def __init__(self):
        super().__init__('autonomous_aoa')
        self.publisher_ = self.create_publisher(Twist, '/model/driftbot/cmd_vel', 10)
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        
        # --- AUTONOMOUS PARAMETERS ---
        self.cruise_speed = 8.0     # Safe testing speed (m/s)
        self.max_steer = 0.45       # Matching your chassis limits
        self.warning_distance = 3.5 # Distance to start swerving (meters)
        self.critical_distance = 1.5 # Distance to slam the brakes (meters)
        
        self.current_speed = 0.0
        self.current_steer = 0.0
        
        self.get_logger().info("🧠 Driftbot AOA Node Online. Scanning for obstacles...")

    def scan_callback(self, msg):
        # The Lidar sweeps 360 samples from -pi to +pi. 
        # Index 180 is dead center (straight ahead).
        
        # Helper function to filter out infinite/NaN errors and find the closest object
        def get_min_dist(slice_start, slice_end):
            valid_ranges = [r for r in msg.ranges[slice_start:slice_end] if not math.isinf(r) and not math.isnan(r)]
            return min(valid_ranges) if valid_ranges else 10.0

        # Define visual zones
        front_dist = get_min_dist(160, 200) # Roughly +/- 20 degrees from center
        left_dist = get_min_dist(200, 260)
        right_dist = get_min_dist(100, 160)

        twist = Twist()

        # --- AOA STATE MACHINE LOGIC ---
        if front_dist > self.warning_distance:
            # 🟢 ALL CLEAR: Floor it and straighten out
            self.current_speed = min(self.current_speed + 0.5, self.cruise_speed)
            self.current_steer = 0.0 
            state = "CRUISING"
            
        elif front_dist > self.critical_distance:
            # 🟡 OBSTACLE DETECTED: Tap the brakes and swerve to the open side
            self.current_speed = max(self.current_speed - 1.0, 3.0)
            
            if left_dist > right_dist:
                self.current_steer = self.max_steer # Swerve Left
                state = "SWERVING LEFT"
            else:
                self.current_steer = -self.max_steer # Swerve Right
                state = "SWERVING RIGHT"
                
        else:
            # 🔴 CRITICAL: Too close! Slam it into reverse!
            self.current_speed = -4.0 
            # Invert steering for reverse out of corners
            self.current_steer = -self.max_steer if left_dist > right_dist else self.max_steer 
            state = "EMERGENCY REVERSE"

        # Fire the commands to the Gazebo physics engine
        twist.linear.x = float(self.current_speed)
        twist.angular.z = float(self.current_steer)
        self.publisher_.publish(twist)

        # Live telemetry dashboard
        print(f"\rState: {state:<17} | Front Gap: {front_dist:4.1f}m | SPD: {self.current_speed:5.1f} | STR: {self.current_steer:4.2f}   ", end="")

def main(args=None):
    rclpy.init(args=args)
    aoa_node = AutonomousAOA()
    try:
        rclpy.spin(aoa_node)
    except KeyboardInterrupt:
        pass
    finally:
        # Send a kill-switch command to stop the bot when you press Ctrl+C
        stop_twist = Twist()
        aoa_node.publisher_.publish(stop_twist)
        aoa_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()