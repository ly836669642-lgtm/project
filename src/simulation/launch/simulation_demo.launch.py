from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def static_tf(name, args):
    return Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=name,
        arguments=[str(a) for a in args],
        output='screen',
    )


def generate_launch_description():
    config_file = LaunchConfiguration('config_file')
    default_config = PathJoinSubstitution([
        FindPackageShare('simulation'),
        'unity_sim', 'Build_Ubuntu', 'AD_Sim_Data', 'StreamingAssets', 'simulation_config.json'
    ])

    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=default_config),
        Node(package='simulation', executable='JSON_param_reader', name='JSON_param_reader', output='screen', parameters=[{'config_file': config_file}]),
        Node(package='simulation', executable='unity_TCP_stream_receiver', name='Unity_ROS_message_Rx', output='screen', parameters=[{'config_file': config_file, 'bind_host': '0.0.0.0'}]),
        Node(package='simulation', executable='ROS_command_transmitter', name='ROS_Unity_command_Tx', output='screen', parameters=[{'config_file': config_file}]),

        static_tf('tf_nominal_center', [0, 0, -0.7180116, 0, 0, 0, 1, 'OurCar/INS', 'OurCar/Center']),
        static_tf('tf_sensors', [0.8199998, 0, 1.269, 0, 0, 0, 1, 'OurCar/Center', 'OurCar/Sensors/SensorBase']),
        static_tf('tf_segmentation_camera', [0, 0, 0, 0.5, -0.5, 0.5, -0.5, 'OurCar/Sensors/SensorBase', 'OurCar/Sensors/SegCamera']),
        static_tf('tf_depth_camera', [0, 0, 0, 0.5, -0.5, 0.5, -0.5, 'OurCar/Sensors/SensorBase', 'OurCar/Sensors/DepthCamera']),
        static_tf('tf_RGB_camera_left', [0, 0.2, 0, 0.5, -0.5, 0.5, -0.5, 'OurCar/Sensors/SensorBase', 'OurCar/Sensors/RGBCameraLeft']),
        static_tf('tf_RGB_camera_right', [0, -0.2, 0, 0.5, -0.5, 0.5, -0.5, 'OurCar/Sensors/SensorBase', 'OurCar/Sensors/RGBCameraRight']),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', PathJoinSubstitution([FindPackageShare('simulation'), 'rviz', 'visualization.rviz'])],
            output='screen',
        ),
    ])
