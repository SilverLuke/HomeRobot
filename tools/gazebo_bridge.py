#!/usr/bin/env python3
import socket
import sys
import os
import time
import threading
import math
import re
import random

# Add proto directory to path to import generated messages
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'proto'))
try:
    import messages_pb2
except ImportError:
    print("Error: Could not import messages_pb2. Run protoc first.")
    sys.exit(1)

# Attempt to import Gazebo Sim libraries
try:
    try:
        # Gazebo Ionic (Sim 9)
        from gz.transport14 import Node
        from gz.msgs11.twist_pb2 import Twist
        from gz.msgs11.odometry_pb2 import Odometry
        from gz.msgs11.imu_pb2 import IMU
        from gz.msgs11.laserscan_pb2 import LidarScan as GzLidarScan
        print("Detected Gazebo Ionic (Sim 9) Python bindings.")
    except ImportError:
        # Gazebo Harmonic (Sim 8)
        from gz.transport13 import Node
        from gz.msgs10.twist_pb2 import Twist
        from gz.msgs10.odometry_pb2 import Odometry
        from gz.msgs10.imu_pb2 import IMU
        from gz.msgs10.laserscan_pb2 import LidarScan as GzLidarScan
        print("Detected Gazebo Harmonic (Sim 8) Python bindings.")
    GZ_AVAILABLE = True
except ImportError:
    print("Warning: Gazebo Sim Python bindings (gz-transport/gz-msgs) not found.")
    print("Falling back to mock mode. Use 'nix run github:muellerbernd/gazebo-sim-overlay#gz-ionic' for the full experience.")
    GZ_AVAILABLE = False

# ==========================================
# ROBOT HARDWARE CONSTANTS
# ==========================================
WHEEL_BASE = 0.26        # meters (distance between wheels)
WHEEL_RADIUS = 0.033     # meters (33mm radius)
TICKS_PER_REV = 360.0    # 1 tick per degree (NXT Lego Motors)
# ==========================================

TICKS_PER_METER = TICKS_PER_REV / (2.0 * math.pi * WHEEL_RADIUS)

ZEPHYR_ADDR = "127.0.0.1"
ZEPHYR_TX_PORT = 6005 # Zephyr sends to this port
ZEPHYR_RX_PORT = 6006 # Zephyr listens on this port

import subprocess
import json

class GazeboBridge:
    def __init__(self):
        self.sock_tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock_rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock_rx.bind(("0.0.0.0", ZEPHYR_TX_PORT))
        
        self.running = True
        self.encoder_left = 0
        self.encoder_right = 0
        self.imu_data = None
        self.lidar_scan = None
        self.lidar_active = True
        
        # Differential Drive State for Encoder Simulation
        self.last_x = 0.0
        self.last_y = 0.0
        self.last_theta = 0.0
        self.total_dist_l = 0.0
        self.total_dist_r = 0.0
        self.first_odom = True
        self.target_linear_x = 0.0
        self.target_angular_z = 0.0
        
        # Constants from SDF
        self.WHEEL_BASE = 0.26  # meters
        self.TICKS_PER_METER = 5780.0 # 1200 ticks per rev, radius 0.033m -> 1200/(2*pi*0.033)
        
        if GZ_AVAILABLE:
            try:
                self.gz_node = Node()
                # Publisher for velocity commands
                self.gz_pub_cmd_vel = self.gz_node.advertise('/model/homerobot/cmd_vel', Twist)
                
                # Subscribers for sensor data
                self.gz_node.subscribe(Odometry, '/model/homerobot/odometry', self.on_gz_odom)
                self.gz_node.subscribe(IMU, '/model/homerobot/imu', self.on_gz_imu)
                self.gz_node.subscribe(GzLidarScan, '/model/homerobot/lidar', self.on_gz_lidar)
                print("Gazebo Sim API connection initialized.")
                self.use_cli_fallback = False
            except Exception as e:
                print(f"Gazebo Sim API initialization failed: {e}")
                self.use_cli_fallback = True
        else:
            self.use_cli_fallback = True
            print("Falling back to CLI/Mock mode.")

        print(f"UDP Bridge started. Listening on UDP {ZEPHYR_TX_PORT}, Sending to {ZEPHYR_RX_PORT}")

    def on_gz_odom(self, msg):
        pos = msg.pose.position
        ori = msg.pose.orientation
        
        # Convert quaternion to yaw (theta)
        siny_cosp = 2 * (ori.w * ori.z + ori.x * ori.y)
        cosy_cosp = 1 - 2 * (ori.y * ori.y + ori.z * ori.z)
        theta = math.atan2(siny_cosp, cosy_cosp)

        if self.first_odom:
            self.last_x = pos.x
            self.last_y = pos.y
            self.last_theta = theta
            self.first_odom = False
            return

        # Calculate deltas
        dx = pos.x - self.last_x
        dy = pos.y - self.last_y
        dtheta = theta - self.last_theta
        
        # Normalize dtheta to [-pi, pi]
        while dtheta > math.pi: dtheta -= 2 * math.pi
        while dtheta < -math.pi: dtheta += 2 * math.pi

        # Fix for drift at rest: Project displacement onto the heading vector
        # This allows tiny physics jitters to cancel out (forward/backward)
        avg_theta = self.last_theta + (dtheta / 2.0)
        d_center = dx * math.cos(avg_theta) + dy * math.sin(avg_theta)
        
        # Deadband: ignore extremely small movements (< 0.1mm) to stop resting flicker
        if abs(d_center) < 0.0001 and abs(dtheta) < 0.0001:
            self.last_x, self.last_y, self.last_theta = pos.x, pos.y, theta
            return

        # Differential kinematics:
        dist_l = d_center - (dtheta * WHEEL_BASE / 2.0)
        dist_r = d_center + (dtheta * WHEEL_BASE / 2.0)

        self.total_dist_l += dist_l
        self.total_dist_r += dist_r
        
        self.encoder_left = int(self.total_dist_l * TICKS_PER_METER)
        self.encoder_right = int(self.total_dist_r * TICKS_PER_METER)

        self.last_x, self.last_y, self.last_theta = pos.x, pos.y, theta

    def on_gz_imu(self, msg):
        self.imu_data = msg

    def on_gz_lidar(self, msg):
        self.lidar_scan = msg

    def rx_loop(self):
        """Receives commands from Zephyr and forwards to Gazebo"""
        while self.running:
            try:
                data, addr = self.sock_rx.recvfrom(1024)
                
                # Try to parse as envelope first
                msg = messages_pb2.ServerToRobotMessage()
                lp, rp = None, None
                
                try:
                    msg.ParseFromString(data)
                    if msg.HasField('motor_move'):
                        cmd = msg.motor_move
                        lp = cmd.left_power * cmd.left_angle
                        rp = cmd.right_power * cmd.right_angle
                    elif msg.HasField('stop_all'):
                        lp, rp = 0, 0
                    else:
                        raise ValueError("Fallback to raw command")
                except:
                    # Fallback to raw MotorMoveCommand (Current Firmware behavior)
                    cmd = messages_pb2.MotorMoveCommand()
                    cmd.ParseFromString(data)
                    lp = cmd.left_power * cmd.left_angle
                    rp = cmd.right_power * cmd.right_angle
                
                if lp is not None and rp is not None:
                    self.target_linear_x = (lp + rp) / 200.0 
                    self.target_angular_z = (rp - lp) / 100.0
                
            except Exception as e:
                print(f"RX Error: {e}")
                break

    def gz_publish_loop(self):
        """Continuous heartbeat for Gazebo to prevent DiffDrive timeout"""
        env = os.environ.copy()
        env["GZ_IP"] = "127.0.0.1"
        env["GZ_PARTITION"] = "homerobot_sim"

        last_linear = -999.0
        last_angular = -999.0
        pub_proc = None

        while self.running:
            # If target changed, restart publisher
            if self.target_linear_x != last_linear or self.target_angular_z != last_angular:
                if pub_proc:
                    pub_proc.kill()
                    pub_proc = None
                
                # Start new publisher for non-zero or for the first stop command
                if self.target_linear_x != 0 or self.target_angular_z != 0 or last_linear != 0 or last_angular != 0:
                    cmd_str = f'GZ_IP=127.0.0.1 GZ_PARTITION=homerobot_sim gz topic -t /model/homerobot/cmd_vel -m gz.msgs.Twist -p "linear: {{x: {self.target_linear_x}}}, angular: {{z: {self.target_angular_z}}}"'
                    try:
                        pub_proc = subprocess.Popen(cmd_str, shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, env=env)
                    except Exception as e:
                        print(f"[BRIDGE] Publisher Error: {e}")
                
                last_linear = self.target_linear_x
                last_angular = self.target_angular_z

            time.sleep(0.2) # 5Hz check

        if pub_proc:
            pub_proc.kill()

    def poll_gz_sensors(self):
        """Background thread to poll sensors via CLI if transport is missing"""
        env = os.environ.copy()
        env["GZ_IP"] = "127.0.0.1"
        env["GZ_PARTITION"] = "homerobot_sim"
        
        # Ensure resource path is there
        if "simulation" not in env.get("GZ_SIM_RESOURCE_PATH", ""):
             env["GZ_SIM_RESOURCE_PATH"] = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "simulation")) + ":" + env.get("GZ_SIM_RESOURCE_PATH", "")

        while self.running:
            if hasattr(self, 'use_cli_fallback') and self.use_cli_fallback:
                # 1. POLL ODOMETRY
                try:
                    res = subprocess.check_output('gz topic -t /model/homerobot/odometry -e -n 1', 
                                                 shell=True, timeout=1.0, env=env).decode()
                    if "position {" in res:
                        # Extract x, y from position
                        x_match = re.search(r'x:\s*([-0-9.eE+]+)', res)
                        y_match = re.search(r'y:\s*([-0-9.eE+]+)', res)
                        if x_match and y_match:
                            x, y = float(x_match.group(1)), float(y_match.group(1))
                            
                            # Extract orientation quaternion
                            oz_match = re.search(r'orientation\s*\{[^}]*z:\s*([-0-9.eE+]+)', res, re.DOTALL)
                            ow_match = re.search(r'orientation\s*\{[^}]*w:\s*([-0-9.eE+]+)', res, re.DOTALL)
                            ox_match = re.search(r'orientation\s*\{[^}]*x:\s*([-0-9.eE+]+)', res, re.DOTALL)
                            oy_match = re.search(r'orientation\s*\{[^}]*y:\s*([-0-9.eE+]+)', res, re.DOTALL)
                            
                            if oz_match and ow_match:
                                oz, ow = float(oz_match.group(1)), float(ow_match.group(1))
                                ox = float(ox_match.group(1)) if ox_match else 0.0
                                oy = float(oy_match.group(1)) if oy_match else 0.0
                                
                                siny_cosp = 2 * (ow * oz + ox * oy)
                                cosy_cosp = 1 - 2 * (oy * oy + oz * oz)
                                theta = math.atan2(siny_cosp, cosy_cosp)
                                
                                if self.first_odom:
                                    self.last_x, self.last_y, self.last_theta = x, y, theta
                                    self.first_odom = False
                                else:
                                    dx, dy = x - self.last_x, y - self.last_y
                                    dtheta = theta - self.last_theta
                                    while dtheta > math.pi: dtheta -= 2 * math.pi
                                    while dtheta < -math.pi: dtheta += 2 * math.pi

                                    avg_theta = self.last_theta + (dtheta / 2.0)
                                    d_center = dx * math.cos(avg_theta) + dy * math.sin(avg_theta)

                                    if not (abs(d_center) < 0.0001 and abs(dtheta) < 0.0001):
                                        dist_l = d_center - (dtheta * WHEEL_BASE / 2.0)
                                        dist_r = d_center + (dtheta * WHEEL_BASE / 2.0)
                                        self.total_dist_l += dist_l
                                        self.total_dist_r += dist_r
                                        self.encoder_left = int(self.total_dist_l * TICKS_PER_METER)
                                        self.encoder_right = int(self.total_dist_r * TICKS_PER_METER)
                                        self.last_x, self.last_y, self.last_theta = x, y, theta
                except Exception as e:
                    pass

                # 2. POLL LIDAR
                if hasattr(self, 'lidar_active') and self.lidar_active:
                    try:
                        lidar_res = subprocess.check_output('gz topic -t /model/homerobot/lidar -e -n 1', 
                                                           shell=True, timeout=1.0, env=env).decode()
                        
                        processed_ranges = []
                        # Try bracket format first
                        bracket_match = re.search(r'ranges:\s+\[(.*?)\]', lidar_res)
                        if bracket_match:
                            range_list = bracket_match.group(1).split(',')
                            for r in range_list:
                                try:
                                    val = float(r.strip())
                                    processed_ranges.append(0.0 if math.isinf(val) or math.isnan(val) else val)
                                except: processed_ranges.append(0.0)
                        else:
                            # Try multi-line format
                            ranges = re.findall(r'ranges:\s+([-+0-9.eE|inf|nan]+)', lidar_res)
                            for r in ranges:
                                try:
                                    val = float(r)
                                    processed_ranges.append(0.0 if math.isinf(val) or math.isnan(val) else val)
                                except: processed_ranges.append(0.0)
                        
                        if processed_ranges:
                            class MockLidar: pass
                            ml = MockLidar()
                            ml.ranges = processed_ranges
                            self.lidar_scan = ml
                    except Exception as e:
                        pass
            
            time.sleep(0.1) # 10Hz polling loop

    def tx_loop(self):
        """Sends Telemetry to Zephyr"""
        while self.running:
            try:
                telemetry = messages_pb2.Telemetry()
                telemetry.encoder_left = self.encoder_left
                telemetry.encoder_right = self.encoder_right
                
                # IMU Data
                if self.imu_data:
                    telemetry.imu.acceleration.x = self.imu_data.linear_acceleration.x
                    telemetry.imu.acceleration.y = self.imu_data.linear_acceleration.y
                    telemetry.imu.acceleration.z = self.imu_data.linear_acceleration.z
                    telemetry.imu.gyroscope.x = self.imu_data.angular_velocity.x
                    telemetry.imu.gyroscope.y = self.imu_data.angular_velocity.y
                    telemetry.imu.gyroscope.z = self.imu_data.angular_velocity.z
                else:
                    # Mock IMU
                    telemetry.imu.acceleration.z = 9.81
                
                # Lidar Data
                if self.lidar_scan:
                    # Convert Gazebo LaserScan to our Protobuf format
                    try:
                        count = len(self.lidar_scan.ranges)
                        for i, dist in enumerate(self.lidar_scan.ranges):
                            # Calculate the angle based on index (0 to 360)
                            base_angle = (i / float(count)) * 360.0
                            # Add random angular noise (e.g., +/- 0.5 degrees)
                            angle_jitter = random.uniform(-0.5, 0.5)
                            
                            p = telemetry.lidar.points.add()
                            p.angle_deg = base_angle + angle_jitter
                            p.distance_mm = float(dist * 1000.0)
                            p.quality = 15
                            p.scan_completed = (i == count - 1)
                    except Exception as e:
                        print(f"Lidar Conversion Error: {e}")
                
                try:
                    data = telemetry.SerializeToString()
                    self.sock_tx.sendto(data, (ZEPHYR_ADDR, ZEPHYR_RX_PORT))
                except Exception as e:
                    print(f"Socket TX Error: {e}")
                
                time.sleep(0.1) # 10Hz telemetry
            except Exception as e:
                import traceback
                print(f"TX Loop Fatal Error: {e}")
                traceback.print_exc()
                break

    def run(self):
        rx_thread = threading.Thread(target=self.rx_loop)
        tx_thread = threading.Thread(target=self.tx_loop)
        poll_thread = threading.Thread(target=self.poll_gz_sensors)
        pub_thread = threading.Thread(target=self.gz_publish_loop)
        rx_thread.daemon = True
        tx_thread.daemon = True
        poll_thread.daemon = True
        pub_thread.daemon = True
        rx_thread.start()
        tx_thread.start()
        poll_thread.start()
        pub_thread.start()
        
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            print("Shutting down bridge...")
            self.running = False
            self.sock_rx.close()

if __name__ == "__main__":
    bridge = GazeboBridge()
    bridge.run()
