from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


UOS_TEST_PATH = 'path/UOS_test/gps_20260906_combined_straight_left_waypoints_index_imu_axes.txt'


def launch_setup(context, *args, **kwargs):
    mode = LaunchConfiguration('mode').perform(context).strip().lower()
    use_serial = LaunchConfiguration('use_serial').perform(context).strip().lower()
    carla = LaunchConfiguration('carla').perform(context).strip().lower()
    serial_enabled = use_serial in ('true', '1', 'yes', 'on')
    use_carla = carla in ('true', '1', 'yes', 'on')

    if mode == 'final':
        mode_name = 'final'
        path_in = UOS_TEST_PATH
        path_out = UOS_TEST_PATH

        velocity_scale_base = (
            '{"0": 20.0, '
            '"1": 18.0, '
            '"2": 18.0, '
            '"3": 20.0, '
            '"4": 20.0}'
        )

    elif mode == 'pre':
        mode_name = 'pre'
        path_in = UOS_TEST_PATH
        path_out = UOS_TEST_PATH

        velocity_scale_base = (
            '{"0": 20.0, '
            '"1": 20.0, '
            '"2": 20.0, '
            '"3": 20.0}'
        )

    else:
        raise RuntimeError(
            f"지원하지 않는 mode입니다: {mode}. "
            "final 또는 pre를 사용해야 합니다."
        )

    planner_vector_node = Node(
        package='autocarz',
        executable='main.py',  # dual GPS backup path
        # executable='main_isro.py',
        name='planner_vector',
        output='screen',
        parameters=[
            {
                'mode.name': mode_name,
                'mode.pathIn': path_in,
                'mode.pathOut': path_out,
                'mode.velocity_scale_base':
                    velocity_scale_base,

                # UOS 경로 파일을 생성할 때 사용한 평면좌표 오프셋.
                # 다른 시험장에서는 launch 인자로만 교체할 수 있다.
                'coordinates.map_offset_x':
                    ParameterValue(
                        LaunchConfiguration('map_offset_x'),
                        value_type=float,
                    ),
                'coordinates.map_offset_y':
                    ParameterValue(
                        LaunchConfiguration('map_offset_y'),
                        value_type=float,
                    ),

                # PIM222A 전용 안전 게이트. PVA/IMU는 각각 50/100 Hz이며,
                # RTK+INS가 연속 2초 안정된 뒤에만 제어를 다시 허용한다.
                'pim222a.gps_timeout_sec': 0.3,
                'pim222a.imu_timeout_sec': 0.2,
                'pim222a.status_timeout_sec': 0.3,
                'pim222a.max_horizontal_covariance': 0.005,
                'pim222a.max_yaw_covariance': 0.01,
                'pim222a.max_time_since_update': 2,
                'pim222a.allowed_position_types': [56],
                'pim222a.allowed_ins_statuses': [3, 7],
                'pim222a.allowed_alignment_states': [2, 3],
                'pim222a.bad_cycles_to_stop': 2,
                'pim222a.good_cycles_to_resume': 20,
            },
        ],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz_lidarm',
        arguments=[
            '-d',
            [
                FindPackageShare('autocarz'),
                '/rviz/autocarz.rviz',
            ],
        ],
        output='screen',
    )

    odom_map_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='link1_broadcaster',
        arguments=[
            '--x',
            '0',
            '--y',
            '0',
            '--z',
            '0',
            '--qx',
            '0',
            '--qy',
            '0',
            '--qz',
            '0',
            '--qw',
            '1',
            '--frame-id',
            'odom',
            '--child-frame-id',
            'map',
        ],
        output='screen',
    )

    base_velodyne_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='link2_broadcaster',
        arguments=[
            '--x',
            '0',
            '--y',
            '0',
            '--z',
            '0',
            '--qx',
            '0',
            '--qy',
            '0',
            '--qz',
            '0',
            '--qw',
            '1',
            '--frame-id',
            'base_link',
            '--child-frame-id',
            'velodyne',
        ],
        output='screen',
    )

    # Livox 드라이버의 PointCloud2 frame_id는 "livox_frame"이다.
    # 실제 장착 위치를 측정하기 전에도 TF 트리가 끊기지 않도록 기본값은
    # base_link와 동일하게 두되, 외부 파라미터로 6-DoF 보정할 수 있게 한다.
    base_livox_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='livox_tf_broadcaster',
        arguments=[
            '--x',
            LaunchConfiguration('livox_x'),
            '--y',
            LaunchConfiguration('livox_y'),
            '--z',
            LaunchConfiguration('livox_z'),
            '--roll',
            LaunchConfiguration('livox_roll'),
            '--pitch',
            LaunchConfiguration('livox_pitch'),
            '--yaw',
            LaunchConfiguration('livox_yaw'),
            '--frame-id',
            'base_link',
            '--child-frame-id',
            'livox_frame',
        ],
        output='screen',
    )

    nodes = [
        planner_vector_node,
        rviz_node,
        odom_map_tf_node,
        base_velodyne_tf_node,
        base_livox_tf_node,
    ]

    if use_carla:
        nodes.append(
            Node(
                package='autocarz_carla_adapter',
                executable='carla_real_topic_adapter',
                name='carla_real_topic_adapter',
                output='screen',
                parameters=[
                    {
                        'carla_odom_topic':
                            LaunchConfiguration('carla_odom_topic'),
                        'carla_control_topic':
                            LaunchConfiguration('carla_control_topic'),
                        'steering_scale':
                            ParameterValue(
                                LaunchConfiguration('carla_steering_scale'),
                                value_type=float,
                            ),
                        'max_speed_mps':
                            ParameterValue(
                                LaunchConfiguration('carla_max_speed_mps'),
                                value_type=float,
                            ),
                        'publish_lidar_relays': False,
                        'publish_erp42_state': not serial_enabled,
                    },
                ],
            )
        )

    if serial_enabled:
        nodes.append(
            Node(
                package='autocarz',
                executable='serialControl.py',
                name='serialControl',
                output='screen',
                parameters=[
                    {
                        'port': LaunchConfiguration('serial_port'),
                        'baud': LaunchConfiguration('serial_baud'),
                    },
                ],
            )
        )

    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'mode',
                default_value='final',
                description=(
                    'competition mode: final | pre'
                ),
            ),

            DeclareLaunchArgument(
                'carla',
                default_value='false',
                description=(
                    'start CARLA topic adapter for direct CARLA testing'
                ),
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
                description=(
                    'steering sign/scale from autocarz CtrlCmd to CARLA'
                ),
            ),

            DeclareLaunchArgument(
                'carla_max_speed_mps',
                default_value='3.0',
                description='maximum CARLA test speed in m/s',
            ),

            DeclareLaunchArgument(
                'use_serial',
                default_value='false',
                description=(
                    'start ERP42 serialControl.py. Use true on the real car.'
                ),
            ),

            DeclareLaunchArgument(
                'serial_port',
                default_value='/dev/ttyUSB0',
                description='ERP42 serial port used when use_serial is true',
            ),

            DeclareLaunchArgument(
                'serial_baud',
                default_value='115200',
                description='ERP42 serial baud rate used when use_serial is true',
            ),

            DeclareLaunchArgument(
                'map_offset_x',
                default_value='-125320.0',
                description='x offset used by the UOS GPS-to-map conversion',
            ),

            DeclareLaunchArgument(
                'map_offset_y',
                default_value='136746.0',
                description='y offset used by the UOS GPS-to-map conversion',
            ),

            DeclareLaunchArgument(
                'livox_x',
                default_value='0.0',
                description='Livox x offset from base_link in metres',
            ),

            DeclareLaunchArgument(
                'livox_y',
                default_value='0.0',
                description='Livox y offset from base_link in metres',
            ),

            DeclareLaunchArgument(
                'livox_z',
                default_value='0.0',
                description='Livox z offset from base_link in metres',
            ),

            DeclareLaunchArgument(
                'livox_roll',
                default_value='0.0',
                description='Livox roll offset from base_link in radians',
            ),

            DeclareLaunchArgument(
                'livox_pitch',
                default_value='0.0',
                description='Livox pitch offset from base_link in radians',
            ),

            DeclareLaunchArgument(
                'livox_yaw',
                default_value='0.0',
                description='Livox yaw offset from base_link in radians',
            ),

            OpaqueFunction(
                function=launch_setup,
            ),
        ]
    )
