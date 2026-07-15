# ROS1 → ROS2 Jazzy porting notes

## Main changes

- Catkin was replaced by `ament_cmake`.
- `roscpp` APIs were replaced by `rclcpp`.
- `sensor_msgs::Image`, `sensor_msgs::CameraInfo`, `sensor_msgs::Imu`, `geometry_msgs::*` were replaced by ROS2 `*.msg::*` types.
- The generated `VehicleControl` message now uses ROS2-compliant lowercase field names: `throttle`, `steering`, `brake`, `reserved`.
- `libsocket` was removed. TCP and UDP transport now use POSIX sockets directly.
- ROS1 XML launch files were kept as `*.ros1` references and replaced by ROS2 Python launch files.
- ROS2 parameters are node-local. The old global ROS parameter server pattern was replaced by having the receiver/transmitter read the config file directly.

## Implemented Unity message types

- `UNITY_STATE = 0`
- `UNITY_CAMERA = 1`
- `UNITY_IMU = 2`
- `UNITY_DEPTH = 3`

`UNITY_FISHEYE` and `UNITY_DETECTIONS` were present in the enum but not implemented in the original C++ bridge and remain unimplemented here.

## Expected build command

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select simulation
```

## Important runtime note

Unity connects as TCP client to the ROS2 receiver. If Unity and ROS2 are not in the same network namespace, do not use `localhost` in Unity's sensor config. Use the ROS2 host IP address and make sure the receiver binds to `0.0.0.0`.


## Fix notes for runtime library and config handling

This package now builds the internal TCP helper code (`tcpstreamreader`, `tcpimage`) as static libraries.
That avoids runtime loader errors such as:

```text
error while loading shared libraries: libtcpimage.so: cannot open shared object file
```

`JSON_param_reader` is a helper/debug executable. If `config_file` is not found, it now logs a warning and exits successfully, while the other nodes continue with their ROS parameters/default values.

Do not launch with the literal placeholder `/path/to/simulation_config.json`. Either omit the argument or pass the actual Unity `StreamingAssets/simulation_config.json` path.
