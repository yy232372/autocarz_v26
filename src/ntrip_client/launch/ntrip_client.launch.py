from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument('host', default_value='RTS2.ngii.go.kr'),
        DeclareLaunchArgument('port', default_value='2101'),
        DeclareLaunchArgument('mountpoint', default_value='VRS-RTCM32'),
        DeclareLaunchArgument('authenticate', default_value='true'),
        DeclareLaunchArgument('username', default_value=''),
        DeclareLaunchArgument('password', default_value=''),
        DeclareLaunchArgument('ntrip_version', default_value=''),
        DeclareLaunchArgument('ssl', default_value='false'),
        DeclareLaunchArgument('rtcm_topic', default_value='/ntrip_client/rtcm'),
        DeclareLaunchArgument('nmea_topic', default_value='/ublox1/nmea'),
    ]
    node = Node(
        package='ntrip_client',
        executable='ntrip_ros.py',
        namespace='ntrip_client',
        name='ntrip_client_node',
        output='screen',
        parameters=[{
            'host': LaunchConfiguration('host'),
            'port': ParameterValue(LaunchConfiguration('port'), value_type=int),
            'mountpoint': LaunchConfiguration('mountpoint'),
            'authenticate': ParameterValue(LaunchConfiguration('authenticate'), value_type=bool),
            'username': LaunchConfiguration('username'),
            'password': LaunchConfiguration('password'),
            'ntrip_version': LaunchConfiguration('ntrip_version'),
            'ssl': ParameterValue(LaunchConfiguration('ssl'), value_type=bool),
        }],
        remappings=[
            ('rtcm', LaunchConfiguration('rtcm_topic')),
            ('nmea', LaunchConfiguration('nmea_topic')),
        ],
    )
    return LaunchDescription(arguments + [node])
