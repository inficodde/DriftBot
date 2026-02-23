#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, select, termios, tty
import os

# --- RACING PARAMETERS ---
MAX_SPEED = 25.0       
BASE_STEER_STEP = 0.15 
THROTTLE_STEP = 2.0    
LOOP_RATE = 0.05       

class ProRacingTeleop(Node):
    def __init__(self):
        super().__init__('pro_racing_teleop')
        self.pub = self.create_publisher(Twist, '/model/driftbot/cmd_vel', 10)
        self.timer = self.create_timer(LOOP_RATE, self.loop)
        self.speed = 0.0
        self.turn = 0.0
        self.settings = termios.tcgetattr(sys.stdin)

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], LOOP_RATE)
        key = sys.stdin.read(1) if rlist else ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def loop(self):
        key = self.get_key()
        
        # 1. Base Speed-Dependent Multiplier
        steer_mult = max(0.3, 1.0 - (abs(self.speed) / MAX_SPEED) * 0.7)
        
        # 2. Reverse Gear Stability Fix
        if self.speed < 0:
            steer_mult *= 0.6

        current_steer_step = BASE_STEER_STEP * steer_mult

        # --- INPUT PROCESSING ---
        if key == 'w': 
            self.speed = min(self.speed + THROTTLE_STEP, MAX_SPEED)
        elif key == 's': 
            self.speed = max(self.speed - THROTTLE_STEP, -MAX_SPEED)
        elif key == 'a': 
            self.turn = min(self.turn + current_steer_step, 0.45)
        elif key == 'd': 
            self.turn = max(self.turn - current_steer_step, -0.45)
        elif key == 'r': 
            self.turn = 0.0
        elif key == ' ': 
            self.speed = 0.0
        elif key == 'q' or key == '\x03': 
            sys.exit()

        if key == '': 
            self.speed *= 0.98

        twist = Twist()
        twist.linear.x = float(self.speed)
        twist.angular.z = float(self.turn)
        self.pub.publish(twist)
        
        sys.stdout.write(f"\rSPD: {self.speed:5.1f} | STR: {self.turn:4.2f} | MULT: {steer_mult:4.2f}      ")
        sys.stdout.flush()

def main():
    rclpy.init(); node = ProRacingTeleop()
    os.system('clear'); 
    print("FINAL PERFORMANCE RC ACTIVE\nW/S: Accel | A/D: Dynamic Steer | SPACE: Brake")
    try: rclpy.spin(node)
    except: pass
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__': main()