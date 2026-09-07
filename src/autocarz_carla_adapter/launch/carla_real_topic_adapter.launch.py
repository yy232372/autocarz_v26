from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'carla_odom_topic',
            default_value='/carla/ego_vehicle/odom',
            description='CARLA bridge odometry already expressed in autocarz path coordinates',
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
        DeclareLaunchArgument(
            'publish_lidar_relays',
            default_value='true',
            description='Relay CARLA lidar point clouds to real autocarz lidar topic names',
        ),
        DeclareLaunchArgument(
            'publish_erp42_state',
            default_value='true',
            description='Publish /erp42/state from CARLA odom. Set false when fake_erp42_serial is running.',
        ),
        Node(
            package='autocarz_carla_adapter',
            executable='carla_real_topic_adapter',
            name='carla_real_topic_adapter',
            output='screen',
            parameters=[
                {
                    'carla_odom_topic': LaunchConfiguration('carla_odom_topic'),
                    'carla_control_topic': LaunchConfiguration('carla_control_topic'),
                    'steering_scale': ParameterValue(LaunchConfiguration('carla_steering_scale'), value_type=float),
                    'max_speed_mps': ParameterValue(LaunchConfiguration('carla_max_speed_mps'), value_type=float),
                    'publish_lidar_relays': ParameterValue(LaunchConfiguration('publish_lidar_relays'), value_type=bool),
                    'publish_erp42_state': ParameterValue(LaunchConfiguration('publish_erp42_state'), value_type=bool),
                }
            ],
        ),
    ])
