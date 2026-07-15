import socket
import struct
import time

UNITY_IP = "172.31.240.1"  
UNITY_PORT = 6657

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

print(f"Sending UDP control packets to {UNITY_IP}:{UNITY_PORT}...")

i = 0
while True:
    throttle = 1.0
    steering = 0.5
    brake = 0.0 
    dummy = 0.0 

    packet = struct.pack("ffff", throttle, steering, brake, dummy)
    sock.sendto(packet, (UNITY_IP, UNITY_PORT))

    print(f"Sent: throttle={throttle:.2f}, steering={steering:.2f}, brake={brake:.2f}, dummy={dummy}")
    i += 1
    time.sleep(0.1) 
