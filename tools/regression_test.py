#!/usr/bin/env python3
import socket
import sys
import os
import time
import threading
import math
import subprocess
import re

# Add proto directory to path
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'proto'))
import messages_pb2

class RegressionTest:
    def __init__(self):
        self.ground_truth_x = 0.0
        self.ground_truth_y = 0.0
        self.ground_truth_theta = 0.0
        self.running = True
        
        # We listen to the same topic as the bridge to get ground truth
        self.env = os.environ.copy()
        self.env["GZ_IP"] = "127.0.0.1"
        self.env["GZ_PARTITION"] = "homerobot_sim"

    def get_ground_truth(self):
        try:
            res = subprocess.check_output('gz topic -t /model/homerobot/odometry -e -n 1', 
                                         shell=True, timeout=2.0, env=self.env).decode()
            x_match = re.search(r'x:\s*([-0-9.eE+]+)', res)
            y_match = re.search(r'y:\s*([-0-9.eE+]+)', res)
            if x_match and y_match:
                self.ground_truth_x = float(x_match.group(1))
                self.ground_truth_y = float(y_match.group(1))
                
                # Orientation
                oz_match = re.search(r'orientation\s*\{[^}]*z:\s*([-0-9.eE+]+)', res, re.DOTALL)
                ow_match = re.search(r'orientation\s*\{[^}]*w:\s*([-0-9.eE+]+)', res, re.DOTALL)
                if oz_match and ow_match:
                    oz, ow = float(oz_match.group(1)), float(ow_match.group(1))
                    self.ground_truth_theta = math.atan2(2 * ow * oz, 1 - 2 * (oz * oz))
                return True
        except:
            return False
        return False

    def run_test(self):
        print("=== HomeRobot Regression Test ===")
        print("1. Checking Gazebo connection...")
        if not self.get_ground_truth():
            print("Error: Could not get ground truth from Gazebo. Is the simulation running?")
            return

        start_x, start_y = self.ground_truth_x, self.ground_truth_y
        print(f"Start Pose: x={start_x:.2f}, y={start_y:.2f}, theta={self.ground_truth_theta:.2f}")

        print("2. Commanding 2-meter forward move...")
        # We use cmd_sender logic here or call it
        try:
            subprocess.run(["cargo", "run", "--", "move", "--left", "100", "--right", "100"], 
                           cwd="tools/cmd_sender", timeout=5)
            time.sleep(3) # Move for 3 seconds
            subprocess.run(["cargo", "run", "--", "stop"], 
                           cwd="tools/cmd_sender", timeout=5)
            time.sleep(1) # Wait for physics to settle
        except Exception as e:
            print(f"Command Error: {e}")

        self.get_ground_truth()
        end_x, end_y = self.ground_truth_x, self.ground_truth_y
        dist_traveled = math.sqrt((end_x - start_x)**2 + (end_y - start_y)**2)
        
        print(f"End Pose: x={end_x:.2f}, y={end_y:.2f}")
        print(f"Actual Distance Traveled: {dist_traveled:.3f}m")

        # Validation Logic (Dummy for now since server-side odom isn't fully integrated here)
        # In a real scenario, we'd compare this dist_traveled with the server's reported odom.
        if dist_traveled > 0.5:
            print("PASS: Robot moved significantly.")
        else:
            print("FAIL: Robot did not move as expected.")

if __name__ == "__main__":
    test = RegressionTest()
    test.run_test()
