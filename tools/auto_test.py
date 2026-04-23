#!/usr/bin/env python3
import socket
import sys
import os
import time
import struct

# Add proto directory to path
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'proto'))
try:
    import messages_pb2
except ImportError:
    print("Error: Could not import messages_pb2. Run protoc first.")
    sys.exit(1)

def send_message(sock, payload_field, payload_msg):
    envelope = messages_pb2.ServerToRobotMessage()
    getattr(envelope, payload_field).CopyFrom(payload_msg)
    data = envelope.SerializeToString()
    header = struct.pack(">H", len(data))
    sock.sendall(header + data)

def main():
    print("=== HomeRobot Autonomous Tester ===")
    server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_sock.bind(("0.0.0.0", 12345))
    server_sock.listen(1)
    server_sock.settimeout(30)

    print("Waiting for robot connection on port 12345...")
    try:
        client_sock, addr = server_sock.accept()
        print(f"Robot connected from {addr}")
        client_sock.settimeout(2.0)
    except socket.timeout:
        print("Error: Timeout waiting for robot connection.")
        return

    # Helper function to read telemetry
    def read_encoders():
        try:
            # Read 2-byte length prefix
            header = client_sock.recv(2)
            if not header: return None, None
            length = struct.unpack(">H", header)[0]
            
            data = client_sock.recv(length)
            msg = messages_pb2.RobotToServerMessage()
            msg.ParseFromString(data)

            if msg.HasField('encoders'):
                return msg.encoders.left_encoder, msg.encoders.right_encoder
        except socket.timeout:
            pass
        except Exception as e:
            print(f"Error reading socket: {e}")
        return None, None

    print("\n--- TEST 1: Resting Drift ---")
    print("Monitoring encoders for 3 seconds without moving...")
    start_time = time.time()
    initial_l, initial_r = None, None
    drift_l, drift_r = 0, 0
    
    while time.time() - start_time < 3:
        l, r = read_encoders()
        if l is not None and r is not None:
            if initial_l is None:
                initial_l, initial_r = l, r
                print(f"  Initial Encoders: L={l}, R={r}")
            else:
                drift_l = abs(l - initial_l)
                drift_r = abs(r - initial_r)
                
    print(f"  Final Drift: L={drift_l}, R={drift_r}")
    if drift_l > 5 or drift_r > 5:
        print("  FAILED: Robot drifted at rest!")
        sys.exit(1)
    else:
        print("  PASSED: Encoders stable at rest.")

    print("\n--- TEST 2: Forward Movement ---")
    print("Sending Move Command (L=50, R=50)...")
    move = messages_pb2.MotorMoveCommand()
    move.left_power = 50
    move.left_angle = 1.0
    move.right_power = 50
    move.right_angle = 1.0
    send_message(client_sock, 'motor_move', move)

    start_time = time.time()
    initial_l, initial_r = None, None
    moved = False

    while time.time() - start_time < 5:
        # Keep sending the move command to prevent Gazebo DiffDrive from timing out
        send_message(client_sock, 'motor_move', move)
        
        l, r = read_encoders()
        if l is not None and r is not None:
            if initial_l is None:
                initial_l, initial_r = l, r
                print(f"  Start Encoders: L={l}, R={r}")
            else:
                diff_l = abs(l - initial_l)
                diff_r = abs(r - initial_r)
                if diff_l > 50 and diff_r > 50:
                    print(f"  Current Encoders: L={l}, R={r} (Diff: L={diff_l}, R={diff_r})")
                    moved = True
                    break

    print("Stopping robot...")
    stop = messages_pb2.MotorMoveCommand()
    send_message(client_sock, 'motor_move', stop)

    client_sock.close()
    server_sock.close()

    if moved:
        print("  PASSED: Movement detected.")
        print("\n=== ALL TESTS PASSED ===")
        sys.exit(0)
    else:
        print("  FAILED: No movement detected.")
        sys.exit(1)

if __name__ == "__main__":
    main()