from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = LaunchConfiguration('config_file')
    default_config = PathJoinSubstitution([
        FindPackageShare('simulation'),
        'unity_sim', 'Build_Ubuntu', 'AD_Sim_Data', 'StreamingAssets', 'simulation_config.json'
    ])

    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=default_config),
        Node(
            package='simulation',
            executable='JSON_param_reader',
            name='JSON_param_reader',
            output='screen',
            parameters=[{'config_file': config_file}],
        ),
        Node(
            package='simulation',
            executable='unity_TCP_stream_receiver',
            name='Unity_ROS_message_Rx',
            output='screen',
            parameters=[{'config_file': config_file, 'bind_host': '0.0.0.0'}],
        ),
        Node(
            package='simulation',
            executable='ROS_command_transmitter',
            name='ROS_Unity_command_Tx',
            output='screen',
            parameters=[{'config_file': config_file}],
        ),
    ])
