# TUM Introduction to ROS 2026 — Autonomous Driving Project

Autonomous car driving a fixed ~785 m urban route in a Unity simulator via ROS 2,
obeying traffic lights, avoiding other vehicles, and handling a vehicle-merge and
an emergency-brake event, without leaving the road.

Tested on **Ubuntu 24.04.3 LTS** with **ROS 2 Jazzy Jalisco**.

## 1. Install dependencies

On a completely fresh Ubuntu 24.04 machine you can run the bundled script to
install ROS 2 Jazzy and every apt dependency below in one go:

```bash
bash install_ros2_jazzy.sh
```

Or install ROS 2 Jazzy (desktop) yourself first if you already have some of this:
https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html

Everything else this project needs beyond `ros-jazzy-desktop`:

```bash
sudo apt update
sudo apt install \
  git unzip \
  python3-colcon-common-extensions \
  ros-jazzy-depth-image-proc \
  ros-jazzy-octomap-server \
  libopencv-dev
```

(`python3-colcon-common-extensions`, `ros-jazzy-depth-image-proc`, and
`ros-jazzy-octomap-server` are the only packages actually missing from a bare
`ros-jazzy-desktop` install; `libopencv-dev` is already pulled in transitively
by `ros-jazzy-desktop` but is listed explicitly since `perception` depends on it
directly. `git`/`unzip` are for cloning this repo and unpacking the Unity build
below, not ROS-specific. Everything else — `rclcpp`, `cv_bridge`, `tf2_ros`,
`rviz2`, `rosidl_default_generators`, etc. — already ships with `ros-jazzy-desktop`.)

## 2. Get the code

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
git clone <THIS_REPO_URL> tmp_clone
mv tmp_clone/src/* src/
mv tmp_clone/.gitignore .
rm -rf tmp_clone
```

(Or, if you cloned this repo directly as `~/ros2_ws`, skip this step — the repo's
`src/` is already your workspace's `src/`.)

## 3. Get the Unity simulator binary

The Unity build (~470 MB unzipped) is **not** included in this repository — it is
the unmodified base simulator provided by the course, identical for every student,
so shipping a second copy through git would just waste your repo's storage/bandwidth
quota for no reason.

1. Download **Version 1.2** of the simulator from the course's LRZ Sync&Share link:
   https://syncandshare.lrz.de/getlink/fiKeDVWGnm9d3e4jtayiDR/
   (if that link has since expired, it's also on the course page / Moodle, or ask
   a teammate who already has it).
2. Unzip it so that the following file exists:
   ```
   ~/ros2_ws/src/simulation/unity_sim/Build_v1.2/LinuxBuild.x86_64
   ```
3. Make it executable and verify these two fields in
   `~/ros2_ws/src/simulation/unity_sim/Build_v1.2/LinuxBuild_Data/StreamingAssets/simulation_config.json`:
   ```json
   "spawnIndexData": { "spawnIndex": 0 },
   "controlConfig": { "controllerOverride": false, ... }
   ```
   (`spawnIndex: 0` is the official benchmark start; if `controllerOverride` is
   `true`, Unity never reads the ROS control socket and the car will not move.)

```bash
chmod +x ~/ros2_ws/src/simulation/unity_sim/Build_v1.2/LinuxBuild.x86_64
```

## 4. Build

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```

All 7 packages (`project_interfaces`, `simulation`, `dummy_controller`, `perception`,
`planning`, `control`, `bringup`) should finish with no errors (verified clean build
from an empty `build/`/`install`/`log`, ~30 s).

> If you have Anaconda/Miniconda installed and see Python-version or
> `libffi`/`libtiff` errors, your shell's `PATH`/`LD_LIBRARY_PATH` is probably
> conda-polluted. Run `unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_SHLVL` and put
> `/usr/bin` first in `PATH` before building.

## 5. Run everything

**Single launch file**, starts the bridge, Unity, the full TF tree, perception
(point cloud → OctoMap, obstacle corridor, traffic-light detector), planning,
control, and RViz:

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch bringup main.launch.py
```

By default (`start_enabled:=true`) the controller starts already enabled and the
car begins driving on its own as soon as it has a trajectory and a pose — no
extra command needed. If you want a supervised/timed start instead, launch with
`start_enabled:=false` (the car holds the brake) and release it when ready:

```bash
ros2 launch bringup main.launch.py start_enabled:=false
# ...when ready:
ros2 service call /control/enable std_srvs/srv/SetBool "{data: true}"
```

Useful launch arguments:

| Argument | Default | Meaning |
|---|---|---|
| `use_unity` | `true` | set `false` to skip starting the Unity process |
| `use_rviz` | `true` | set `false` to skip RViz |
| `drive` | `true` | set `false` to start without the controller node at all |
| `start_enabled` | `true` | controller's initial enable state |
| `lane_offset_scale` | `1.0` | 0.0 drives the raw recorded (left-lane) line, for A/B comparison |
| `unity_bin` / `sim_config` | auto-detected | override if the Unity build lives elsewhere |

Example: `ros2 launch bringup main.launch.py use_rviz:=false`

## 6. Package overview

| Package | What it does |
|---|---|
| `simulation` *(course-provided, ported to ROS 2 Jazzy)* | TCP bridge to Unity: publishes true pose/twist, camera/depth images, IMU; UDP command transmitter |
| `dummy_controller` *(course-provided)* | Minimal example controller (not used by `bringup`, kept for reference) |
| `project_interfaces` | Our custom message types: `Trajectory`/`TrajectoryPoint` (planned path with per-point speed), `TrafficLight` (detected signal state) |
| `perception` | `obstacle_guard_node` (depth cloud → forward obstacle corridor distances), `traffic_light_node` (RGB HSV blob detection of the facing signal head — deliberately not using the semantic camera) |
| `planning` | `route_planner_node`: lane-snapped waypoints → curvature-limited speed profile → trajectory, with reactive detour replanning around blocked obstacles |
| `control` | `pure_pursuit_node`: pure-pursuit path tracking + ACC-style gap control, traffic-light stop/go state machine, detour/verify state machine |
| `bringup` | Single launch file (`main.launch.py`) + static TF tree + RViz config wiring everything above together |

A full architecture write-up with figures, the ROS graph, and results is in the
separate project documentation (not this README).

## 7. Required ROS 2 elements implemented ourselves

- **Custom message types** (`project_interfaces/msg/Trajectory`,
  `TrajectoryPoint`, `TrafficLight`) — defined and used throughout planning/control.
- **ROS service**: `/control/enable` (`std_srvs/srv/SetBool`), implemented in
  `control/src/pure_pursuit_node.cpp` — `true` releases the brake and starts
  driving, `false` holds the brake.

## 8. Team & contributions

<!-- TODO: fill in team member names and who worked on which package/node -->

## Troubleshooting

- **Car doesn't move, throttle/brake look fine**: check `controllerOverride: false`
  in `simulation_config.json` (see step 3).
- **"Corridor data stale" warnings right after launch**: normal for the first ~1 s
  while the TCP connection to Unity warms up; persistent staleness means Unity
  isn't publishing depth images (check `ros2 topic hz /OurCar/Sensors/DepthCamera/image_raw`).
- **Stale/ghost processes after killing a launch**: `ros2 launch` can leave child
  nodes running. Before relaunching: `pkill -f "ros2 launch bringup"` and kill any
  leftover `LinuxBuild.x86_64` / `*_node` processes, then
  `rm -f /dev/shm/fastrtps_*`.
- **`ros2 topic list` shows nothing even though the launch log looks fine**: a
  stale `ros2 daemon` from a previous session. Run `ros2 daemon stop` (it
  restarts itself automatically).
