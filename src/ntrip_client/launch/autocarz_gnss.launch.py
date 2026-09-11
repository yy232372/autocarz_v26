from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rtcm_topic = LaunchConfiguration('rtcm_topic')
    return LaunchDescription([
        DeclareLaunchArgument('host', default_value='RTS2.ngii.go.kr'),
        DeclareLaunchArgument('port', default_value='2101'),
        DeclareLaunchArgument('mountpoint', default_value='VRS-RTCM32'),
        DeclareLaunchArgument('username', default_value=''),
        DeclareLaunchArgument('password', default_value=''),
        DeclareLaunchArgument('rtcm_topic', default_value='/ntrip_client/rtcm'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('ublox_gps'), 'launch', 'autocarz_dual_gps.launch.py'
            ])),
            launch_arguments={'rtcm_topic': rtcm_topic}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('ntrip_client'), 'launch', 'ntrip_client.launch.py'
            ])),
            launch_arguments={
                'host': LaunchConfiguration('host'),
                'port': LaunchConfiguration('port'),
                'mountpoint': LaunchConfiguration('mountpoint'),
                'username': LaunchConfiguration('username'),
                'password': LaunchConfiguration('password'),
                'rtcm_topic': rtcm_topic,
                'nmea_topic': '/ublox1/nmea',
            }.items(),
        ),
    ])
