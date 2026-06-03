import socket
import struct
import time
import messages_pb2

def send_move(s, lp, rp, la, ra):
    msg = messages_pb2.ServerToRobotMessage()
    msg.sequence_millis = int(time.time() * 1000) % 0xFFFFFFFF
    msg.motor_move.left_power = lp
    msg.motor_move.right_power = rp
    msg.motor_move.left_angle = la
    msg.motor_move.right_angle = ra
    
    data = msg.SerializeToString()
    header = struct.pack(">H", len(data))
    s.sendall(header + data)

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind(('0.0.0.0', 12345))
s.listen(1)
print("Waiting for robot...")
conn, addr = s.accept()
print(f"Robot connected from {addr}")

try:
    while True:
        print("Sending Move...")
        send_move(conn, 200, 200, 1.0, 1.0)
        time.sleep(0.1)
except KeyboardInterrupt:
    pass
finally:
    conn.close()
    s.close()
