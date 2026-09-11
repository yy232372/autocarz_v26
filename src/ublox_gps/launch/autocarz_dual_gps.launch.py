import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory('ublox_gps'), 'config')
    rtcm_topic = LaunchConfiguration('rtcm_topic')
    nodes = []
    for index in (1, 2):
        nodes.append(Node(
            package='ublox_gps',
            executable='ublox_gps_node',
            namespace=f'ublox{index}',
            name='ublox_gps_node',
            output='screen',
            respawn=True,
            respawn_delay=30.0,
            parameters=[os.path.join(config_dir, f'autocarz_gps{index}.yaml')],
            remappings=[('/rtcm', rtcm_topic)],
        ))
    return LaunchDescription([
        DeclareLaunchArgument('rtcm_topic', default_value='/ntrip_client/rtcm'),
        *nodes,
    ])
