from message_decoders import show_image_live, read_exact
from message_decoders import parse_imu, parse_rgbcamera
from message_decoders import decode_depth_rgb, visualize_depth_grayscale_fixed, parse_depthcamera
from message_decoders import parse_truestate
import numpy as np
import cv2

UNITY_STATE     = 0
UNITY_CAMERA    = 1
UNITY_IMU       = 2
UNITY_DEPTH     = 3
UNITY_FISHEYE   = 4
UNITY_DETECTIONS = 5
MESSAGE_TYPE_COUNT = 6

# def read_header(conn):
#     type_id = struct.unpack('<I', read_exact(conn, 4))[0]
#     ticks = struct.unpack('<Q', read_exact(conn, 8))[0]
#     name = read_string(conn)
#     timestamp = ticks * 1e-7
#     return type_id, timestamp, name


import socket
import struct

HOST = '0.0.0.0'
PORT = 9998

def read_string(sock):
    chars = []
    while True:
        char = sock.recv(1)
        if char == b'\x00':
            break
        chars.append(char)
    return b''.join(chars).decode('ascii')

def main():
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((HOST, PORT))
    server_socket.listen(1)

    print(f"[INFO] Waiting for Unity connection on port {PORT}...")
    conn, addr = server_socket.accept()
    print(f"[INFO] Connected by {addr}")
    try:
        while True:
            # Step 1: header
            magic = struct.unpack('<I', read_exact(conn, 4))[0]
            if magic != 0xDEADC0DE:
                print(f"[ERROR] Magic mismatch: {hex(magic)}")
                continue

            type_id = struct.unpack('<I', read_exact(conn, 4))[0]
            ticks = struct.unpack('<Q', read_exact(conn, 8))[0]
            name = read_string(conn)
            print(f"\n[INFO] Got image from '{name}' | Type: {type_id} | Ticks: {ticks}")
            
            if type_id == UNITY_CAMERA:
                parse_rgbcamera(conn = conn, name = name, timestamp = ticks)
            elif type_id == UNITY_DEPTH:
                parse_depthcamera(conn = conn, name = name, timestamp = ticks)
            elif type_id == UNITY_STATE:
                parse_truestate(conn = conn, name = name, timestamp = ticks, visualize = False)
            # elif type_id == UNITY_DETECTIONS:
            #     parse_segmentationcamera(conn=conn, name = name, timestamp=ticks)

    except Exception as e:
        print(f"[ERROR] {e}")
    finally:
        conn.close()
        server_socket.close()

if __name__ == "__main__":
    main()
