# Introduction to ROS 2 – Course Project Autonomous Driving

## Overview

This repository is part of the "Introduction to ROS" course project **Autonomous Driving**. It contains a Unity-based driving simulation package and a dummy controller to demonstrate how to control the vehicle from ROS 2.

The repository contains two ROS 2 packages:

- `simulation/`
  - Unity bridge and simulation integration package.
  - Receives sensor data from Unity over TCP.
  - Publishes camera, depth, IMU, and state information as ROS 2 topics.
  - Provides the custom message type `simulation/msg/VehicleControl`.

- `dummy_controller/`
  - A simple ROS 2 node that publishes vehicle control commands.
  - Publishes `simulation/msg/VehicleControl` messages to `/car_command`.

## System Requirements

- **Operating System**: Ubuntu 24.04 recommended for ROS 2 Jazzy.
  - Ubuntu 22.04/WSL/Docker may also work if ROS 2 Jazzy is installed correctly, but the recommended platform for Jazzy is Ubuntu 24.04.
- **ROS Version**: ROS 2 Jazzy Jalisco.
- **Required Tools**:
  - `colcon`
  - Git
  - Git LFS, required for Unity binaries and large assets

Install Git LFS with:

```bash
sudo apt update
sudo apt install git-lfs
git lfs install
```

Install common ROS 2 build tools and dependencies:

```bash
sudo apt update
sudo apt install \
  python3-colcon-common-extensions \
  ros-jazzy-rclcpp \
  ros-jazzy-std-msgs \
  ros-jazzy-sensor-msgs \
  ros-jazzy-geometry-msgs \
  ros-jazzy-tf2-ros \
  ros-jazzy-rosidl-default-generators \
  ros-jazzy-ament-index-cpp \
  ros-jazzy-rviz2
```

## Repository Structure

Your ROS 2 workspace should look like this:

```text
your_ros2_workspace/
└── src/
    ├── simulation/
    ├── dummy_controller/
    ├── setup_script.sh
    └── README.md
```

A top-level `CMakeLists.txt` file is not required for ROS 2 workspaces. ROS 2 uses `colcon` to discover packages inside `src/`.

## Getting Started

### 1. Clone or copy the repository into your workspace

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
# clone or copy simulation/ and dummy_controller/ here
```

### 2. Run the setup script

The setup script marks Unity and helper scripts as executable when they exist:

```bash
bash setup_script.sh
```

If the script is located in the workspace root instead of `src/`, run:

```bash
bash src/setup_script.sh
```

### 3. Build the workspace

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

To build only the two project packages:

```bash
colcon build --packages-select simulation dummy_controller
source install/setup.bash
```

## Running the Simulation

Start the Unity bridge and simulation integration package:

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch simulation simulation.launch.py
```

If your launch file accepts a config file argument, you can pass it explicitly:

```bash
ros2 launch simulation simulation.launch.py \
  config_file:=/path/to/simulation_config.json
```

Unlike ROS 1, ROS 2 does not require a separate `roscore` process.

## Running the Dummy Controller Node

In another terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch dummy_controller dummy_controller.launch.py
```

Alternatively, run the node directly:

```bash
ros2 run dummy_controller dummy_controller_node
```

The dummy controller publishes vehicle commands to:

```text
/car_command
```

with message type:

```text
simulation/msg/VehicleControl
```

You can inspect the command topic with:

```bash
ros2 topic echo /car_command
ros2 topic hz /car_command
```

## Useful ROS 2 Debug Commands

List topics:

```bash
ros2 topic list
```

Inspect a topic type:

```bash
ros2 topic info /car_command
```

Show the custom message definition:

```bash
ros2 interface show simulation/msg/VehicleControl
```

View image topics:

```bash
ros2 run rqt_image_view rqt_image_view
```

## Unity TCP Notes

The Unity simulation sends sensor data to the ROS 2 bridge over TCP. The Unity-side `TCPServer.cs` acts as a TCP client for sensor transmission, so the ROS 2 bridge must be running and listening before Unity can connect.

For local native Ubuntu usage, `127.0.0.1:9998` is usually correct.

For Unity on Windows and ROS 2 in WSL2, do not use `localhost` inside Unity. Use the WSL2 IP address shown by:

```bash
hostname -I
```

For Unity and ROS 2 on different machines, use the ROS 2 machine's LAN IP address and make sure the ROS 2 bridge listens on `0.0.0.0`.

## Customizing the Simulation

The simulation package provides configurable parameters such as sensor setup, socket ports, and launch options. See the README inside the `simulation/` package for package-specific configuration details.

## Notes for Students

The original ROS 1 version used:

```bash
catkin build
roscore
roslaunch
rosrun
source devel/setup.bash
```

The ROS 2 Jazzy version uses:

```bash
colcon build
ros2 launch
ros2 run
source install/setup.bash
```

There is no `roscore` in ROS 2.
