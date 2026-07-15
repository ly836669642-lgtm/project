from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='dummy_controller',
            executable='dummy_controller_node',
            name='dummy_controller',
            output='screen',
        ),
    ])
