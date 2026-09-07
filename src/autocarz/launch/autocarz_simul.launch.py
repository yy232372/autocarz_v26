from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='autocarz',
            executable='car_viusal',
            name='car_viusal_in',
            output='screen',
            parameters=[{'in_or_out': 'in'}],
        ),
        Node(
            package='autocarz',
            executable='car_viusal',
            name='car_viusal_out',
            output='screen',
            parameters=[{'in_or_out': 'out'}],
        ),
        Node(
            package='autocarz',
            executable='path_pub.py',
            name='path_pub',
            output='screen',
        ),
    ])
