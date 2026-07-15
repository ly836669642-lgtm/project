import socket
import struct
def read_exact(sock, size):
    buf = b''
    while len(buf) < size:
        part = sock.recv(size - len(buf))
        if not part:
            raise ConnectionError("Socket closed")
        buf += part
    return buf

import cv2
import numpy as np
def show_image_live(image_bytes, width, height, name):
    img_array = np.frombuffer(image_bytes, dtype=np.uint8)
    img_array = img_array.reshape((height, width, 3))

    img_array = cv2.flip(img_array, 0)
    img_bgr = cv2.cvtColor(img_array, cv2.COLOR_RGB2BGR)
    cv2.imshow(name, img_bgr)
    cv2.waitKey(1)



def parse_rgbcamera(conn, name, timestamp):
    is_mono = struct.unpack('<i', read_exact(conn, 4))[0]
    fov = struct.unpack('<f', read_exact(conn, 4))[0]
    far = struct.unpack('<f', read_exact(conn, 4))[0]
    width = struct.unpack('<i', read_exact(conn, 4))[0]
    height = struct.unpack('<i', read_exact(conn, 4))[0]
    print(f"[RGB] {name} @ {timestamp:.3f}s | {width}x{height} | fov: {fov} | mono: {is_mono}")

    # Step 3: image data
    img_len = width * height * 3  # RGB24
    image_bytes = read_exact(conn, img_len)
    show_image_live(image_bytes=image_bytes, width=width, height=height, name = name)

def parse_imu(conn, name, timestamp):
    accel = struct.unpack('<fff', read_exact(conn, 12))
    gyro  = struct.unpack('<fff', read_exact(conn, 12))

    print(f"[IMU] {name} @ {timestamp:.3f}s")
    print(f"  Acceleration [m/s²]: x={accel[0]:.3f}, y={accel[1]:.3f}, z={accel[2]:.3f}")
    print(f"  Angular vel [rad/s]: x={gyro[0]:.3f}, y={gyro[1]:.3f}, z={gyro[2]:.3f}")

def decode_depth_rgb(image_bytes, width, height, max_range=65.535):
    img_array = np.frombuffer(image_bytes, dtype=np.uint8).reshape((height, width, 3))
    img_array = cv2.flip(img_array, 0)

    r = img_array[:, :, 0].astype(np.float32) / 255.0
    g = img_array[:, :, 1].astype(np.float32) / 255.0
    b = img_array[:, :, 2].astype(np.float32) / 255.0

    depth = r + g / 255.0 + b / 65025.0

    depth[depth > 0.99] = 0.0

    return depth * max_range

def visualize_depth_grayscale_fixed(depth, max_range=65.535):
    depth_clipped = np.clip(depth, 0, max_range)
    depth_mask = (depth > 0) & (depth <= max_range)

    normalized = np.zeros_like(depth, dtype=np.float32)
    normalized[depth_mask] = (max_range - depth_clipped[depth_mask]) / max_range
    grayscale = (normalized * 255).astype(np.uint8)

    return grayscale


def parse_depthcamera(conn, name, timestamp):
    is_mono = struct.unpack('<i', read_exact(conn, 4))[0]
    fov = struct.unpack('<f', read_exact(conn, 4))[0]
    far = struct.unpack('<f', read_exact(conn, 4))[0]
    width = struct.unpack('<i', read_exact(conn, 4))[0]
    height = struct.unpack('<i', read_exact(conn, 4))[0]

    print(f"[DEPTH RGB] {name} @ {timestamp:.3f}s | {width}x{height} | fov: {fov} | far: {far}")

    img_len = width * height * 3
    image_bytes = read_exact(conn, img_len)

    depth = decode_depth_rgb(image_bytes, width, height)
    gray = visualize_depth_grayscale_fixed(depth)
    cv2.imshow(name, gray)
    cv2.waitKey(1)

def parse_truestate(conn, name, timestamp, visualize = False, plot_interval = 0.1, max_length = 150):
    position = struct.unpack('<fff', read_exact(conn, 12))
    rotation = struct.unpack('<ffff', read_exact(conn, 16))
    velocity = struct.unpack('<fff', read_exact(conn, 12))
    angular_velocity = struct.unpack('<fff', read_exact(conn, 12))

    print(f"[TrueState] {name} @ {timestamp:.3f}s")
    print(f"  Position: ({position[0]:.2f}, {position[1]:.2f}, {position[2]:.2f})")
    print(f"  Rotation (quat): ({rotation[0]:.3f}, {rotation[1]:.3f}, {rotation[2]:.3f}, {rotation[3]:.3f})")
    print(f"  Linear Velocity: ({velocity[0]:.2f}, {velocity[1]:.2f}, {velocity[2]:.2f})")
    print(f"  Angular Velocity: ({angular_velocity[0]:.2f}, {angular_velocity[1]:.2f}, {angular_velocity[2]:.2f})")
    if visualize:
        update_trajectory(position, plot_interval = plot_interval, max_length=max_length)

import matplotlib.pyplot as plt
from collections import deque
import numpy as np
import time

last_plot_time = 0.0

trajectory_x = None
trajectory_y = None

fig, ax = plt.subplots()
line, = ax.plot([], [], lw=2)

# ax.set_xlim(-10, 10)
# ax.set_ylim(-10, 10)
# ax.set_zlim(0, 20)
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_title('TrueState Trajectory')

def update_trajectory(position, plot_interval = 0.1, max_length = 150):
    global trajectory_x, trajectory_y
    if trajectory_x is None:
        trajectory_x = deque(maxlen = max_length)
        trajectory_y = deque(maxlen = max_length)

    global last_plot_time
    current_time = time.time()
    if current_time - last_plot_time < plot_interval:
        return
    x, z, y = position
    trajectory_x.append(x)
    trajectory_y.append(y)

    line.set_data(trajectory_x, trajectory_y)
    ax.relim()
    ax.autoscale_view()
    plt.draw()
    plt.pause(0.0001)
    last_plot_time = current_time
