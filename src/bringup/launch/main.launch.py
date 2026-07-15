"""Single entry point for the whole system.

Starts, in order: the ROS-Unity bridge (TCP listener first), the Unity
simulator (delayed so the listener is up), the command transmitter, the
static TF tree for all sensors, the perception pipeline (depth image ->
point cloud -> OctoMap, corridor obstacle guard, traffic-light detector),
the route planner, the pure-pursuit controller (gated by 'drive'), and
RViz.

    ros2 launch bringup main.launch.py
    ros2 launch bringup main.launch.py drive:=false          # no controller
    ros2 launch bringup main.launch.py use_unity:=false use_rviz:=false
    ros2 launch bringup main.launch.py unity_bin:=/path/to/LinuxBuild.x86_64
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            TimerAction)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def find_unity_dir():
    """Locate the Unity build in the simulation package's source tree.

    The Unity build is too large to install into the package share, so it
    stays under <workspace>/src/simulation/unity_sim.  The workspace root is
    derived from this package's install location, which works regardless of
    where the workspace was cloned.
    """
    candidates = []
    # <ws>/install/bringup/share/bringup -> <ws>
    ws_root = os.path.normpath(os.path.join(
        get_package_share_directory('bringup'), '..', '..', '..', '..'))
    candidates.append(os.path.join(ws_root, 'src'))
    # Fallback: launch file installed via --symlink-install resolves into src.
    candidates.append(os.path.normpath(os.path.join(
        os.path.dirname(os.path.realpath(__file__)), '..', '..')))
    for src_dir in candidates:
        unity_dir = os.path.join(src_dir, 'simulation', 'unity_sim',
                                 'Build_v1.2')
        if os.path.isfile(os.path.join(unity_dir, 'LinuxBuild.x86_64')):
            return unity_dir
    raise RuntimeError(
        'Unity build not found (looked under: %s). Pass it explicitly: '
        'ros2 launch bringup main.launch.py '
        'unity_bin:=/path/to/LinuxBuild.x86_64 '
        'sim_config:=/path/to/simulation_config.json' % candidates)


def static_tf(name, parent, child, x=0.0, y=0.0, z=0.0,
              qx=0.0, qy=0.0, qz=0.0, qw=1.0):
    return Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=name,
        arguments=[
            '--x', str(x), '--y', str(y), '--z', str(z),
            '--qx', str(qx), '--qy', str(qy), '--qz', str(qz),
            '--qw', str(qw),
            '--frame-id', parent, '--child-frame-id', child,
        ],
    )


def generate_launch_description():
    unity_dir = find_unity_dir()
    unity_bin_default = os.path.join(unity_dir, 'LinuxBuild.x86_64')
    sim_config_default = os.path.join(
        unity_dir, 'LinuxBuild_Data', 'StreamingAssets',
        'simulation_config.json')

    use_unity = LaunchConfiguration('use_unity')
    use_rviz = LaunchConfiguration('use_rviz')
    unity_bin = LaunchConfiguration('unity_bin')
    sim_config = LaunchConfiguration('sim_config')

    # Camera optical frame orientation relative to the body frame (FLU):
    # z forward, x right, y down.
    OPT = {'qx': 0.5, 'qy': -0.5, 'qz': 0.5, 'qw': -0.5}

    return LaunchDescription([
        DeclareLaunchArgument('use_unity', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('drive', default_value='true'),
        # start_enabled:=false starts the controller holding the brake
        # until /control/enable is called - supervised/timed starts
        DeclareLaunchArgument('start_enabled', default_value='true'),
        # multiplier on the per-stretch right-lane shift schedule in the
        # route planner (see lane_offset_zones there). 0.0 drives the
        # recorded line unchanged, for A/B runs.
        DeclareLaunchArgument('lane_offset_scale', default_value='1.0'),
        DeclareLaunchArgument('unity_bin', default_value=unity_bin_default),
        DeclareLaunchArgument('sim_config', default_value=sim_config_default),

        # --- ROS-Unity bridge. The receiver is the TCP listener Unity
        # connects to; it exits whenever Unity disconnects, so it respawns.
        Node(
            package='simulation',
            executable='unity_TCP_stream_receiver',
            name='Unity_ROS_message_Rx',
            parameters=[{'config_file': sim_config,
                         'bind_host': '0.0.0.0'}],
            respawn=True,
            respawn_delay=1.0,
            output='screen',
        ),
        Node(
            package='simulation',
            executable='ROS_command_transmitter',
            name='ROS_Unity_command_Tx',
            parameters=[{'config_file': sim_config}],
            respawn=True,
            respawn_delay=1.0,
            output='screen',
        ),

        # --- Unity simulator, delayed so the bridge is already listening.
        TimerAction(
            period=3.0,
            actions=[ExecuteProcess(
                cmd=[unity_bin],
                condition=IfCondition(use_unity),
                output='log',
            )],
        ),

        # --- Static TF tree: INS -> vehicle center -> sensor rig -> cameras.
        # Offsets follow the vehicle drawings in the task PDF (Fig. 2/3) and
        # the reference values shipped in simulation_demo.launch.py.
        static_tf('tf_center', 'OurCar/INS', 'OurCar/Center', z=-0.7180116),
        static_tf('tf_sensor_base', 'OurCar/Center',
                  'OurCar/Sensors/SensorBase', x=0.8199998, z=1.269),
        static_tf('tf_depth_camera', 'OurCar/Sensors/SensorBase',
                  'OurCar/Sensors/DepthCamera', **OPT),
        static_tf('tf_semantic_camera', 'OurCar/Sensors/SensorBase',
                  'OurCar/Sensors/SemanticCamera', **OPT),
        static_tf('tf_rgb_left', 'OurCar/Sensors/SensorBase',
                  'OurCar/Sensors/RGBCameraLeft', y=0.2, **OPT),
        static_tf('tf_rgb_right', 'OurCar/Sensors/SensorBase',
                  'OurCar/Sensors/RGBCameraRight', y=-0.2, **OPT),

        # --- Perception: depth image -> point cloud ---
        ComposableNodeContainer(
            name='perception_container',
            namespace='perception',
            package='rclcpp_components',
            executable='component_container',
            output='screen',
            composable_node_descriptions=[
                ComposableNode(
                    package='depth_image_proc',
                    plugin='depth_image_proc::PointCloudXyzNode',
                    name='depth_to_cloud',
                    namespace='perception',
                    remappings=[
                        ('image_rect', '/OurCar/Sensors/DepthCamera/image_raw'),
                        ('camera_info', '/OurCar/Sensors/DepthCamera/camera_info'),
                        ('points', '/perception/pcl/points'),
                    ],
                ),
            ],
        ),

        # --- Perception: point cloud -> OctoMap occupancy map ---
        Node(
            package='octomap_server',
            executable='octomap_server_node',
            name='octomap_server',
            namespace='perception/octomap',
            parameters=[{
                'resolution': 0.4,
                'frame_id': 'world',
                'sensor_model.max_range': 50.0,
            }],
            remappings=[('cloud_in', '/perception/pcl/points')],
        ),

        # --- Planning and control ---
        Node(
            package='perception',
            executable='obstacle_guard_node',
            name='obstacle_guard',
            output='screen',
        ),
        Node(
            package='perception',
            executable='traffic_light_node',
            name='traffic_light',
            output='screen',
        ),
        Node(
            package='planning',
            executable='route_planner_node',
            name='route_planner',
            parameters=[{
                'lane_offset_scale': LaunchConfiguration('lane_offset_scale'),
            }],
            output='screen',
        ),
        Node(
            package='control',
            executable='pure_pursuit_node',
            name='pure_pursuit',
            condition=IfCondition(LaunchConfiguration('drive')),
            parameters=[{
                'start_enabled': LaunchConfiguration('start_enabled'),
            }],
            output='screen',
        ),

        # --- Visualization ---
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            condition=IfCondition(use_rviz),
            arguments=['-d', PathJoinSubstitution([
                FindPackageShare('bringup'), 'rviz', 'perception.rviz'])],
            output='log',
        ),
    ])
