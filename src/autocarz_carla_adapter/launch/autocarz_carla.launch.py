from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    mode = LaunchConfiguration('mode')
    carla_odom_topic = LaunchConfiguration('carla_odom_topic')
    carla_control_topic = LaunchConfiguration('carla_control_topic')
    carla_steering_scale = LaunchConfiguration('carla_steering_scale')
    carla_max_speed_mps = LaunchConfiguration('carla_max_speed_mps')

    adapter_node = Node(
        package='autocarz_carla_adapter',
        executable='carla_real_topic_adapter',
        name='carla_real_topic_adapter',
        output='screen',
        parameters=[
            {
                'carla_odom_topic': carla_odom_topic,
                'carla_control_topic': carla_control_topic,
                'steering_scale': ParameterValue(carla_steering_scale, value_type=float),
                'max_speed_mps': ParameterValue(carla_max_speed_mps, value_type=float),
                'publish_lidar_relays': True,
                'publish_erp42_state': True,
            },
        ],
    )

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                FindPackageShare('autocarz'),
                '/launch/autocarz_lidar.launch.py',
            ]
        ),
        launch_arguments={
            'mode': mode,
            'carla': 'false',
        }.items(),
    )

    vector_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                FindPackageShare('autocarz'),
                '/launch/autocarz_vector.launch.py',
            ]
        ),
        launch_arguments={
            'mode': mode,
            'use_serial': 'false',
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'mode',
                default_value='final',
                description='competition mode: final | pre',
            ),
            DeclareLaunchArgument(
                'carla_odom_topic',
                default_value='/carla/ego_vehicle/odom',
                description='CARLA bridge odometry topic',
            ),
            DeclareLaunchArgument(
                'carla_control_topic',
                default_value='/carla/ego_vehicle/ackermann_control_cmd',
                description='CARLA bridge Ackermann control topic',
            ),
            DeclareLaunchArgument(
                'carla_steering_scale',
                default_value='-1.0',
                description='Steering sign/scale from autocarz CtrlCmd to CARLA Ackermann',
            ),
            DeclareLaunchArgument(
                'carla_max_speed_mps',
                default_value='3.0',
                description='Maximum CARLA test speed in m/s',
            ),
            adapter_node,
            lidar_launch,
            vector_launch,
        ]
    )
