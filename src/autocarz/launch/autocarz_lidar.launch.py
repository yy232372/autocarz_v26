from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


UOS_TEST_PATH = 'path/UOS_test/gps_20260906_combined_straight_left_waypoints_index_imu_axes.txt'


def launch_setup(context, *args, **kwargs):
    mode = LaunchConfiguration('mode').perform(context).strip().lower()
    carla = LaunchConfiguration('carla').perform(context).strip().lower()

    if mode == 'final':
        path_in = UOS_TEST_PATH
        path_out = UOS_TEST_PATH

    elif mode == 'pre':
        path_in = UOS_TEST_PATH
        path_out = UOS_TEST_PATH

    else:
        raise RuntimeError(
            f"지원하지 않는 mode입니다: {mode}. "
            "final 또는 pre를 사용해야 합니다."
        )

    use_carla = carla in (
        'true',
        '1',
        'yes',
        'on',
    )

    if use_carla:
        vlp_pointcloud_topic = '/carla/ego_vehicle/lidar_vlp16'
        livox_pointcloud_topic = '/carla/ego_vehicle/lidar_vlp16'
    else:
        vlp_pointcloud_topic = '/velodyne_points'
        livox_pointcloud_topic = '/livox/lidar'

    nodes = []

    # =========================================================================
    # VLP-16 파이프라인
    #
    # /velodyne_points
    #   ├─ /vlp/roi_in
    #   │    └─ /vlp/ransac_in
    #   │          └─ /vlp/cluster_in
    #   └─ /vlp/roi_out
    #        └─ /vlp/ransac_out
    #              └─ /vlp/cluster_out
    # =========================================================================

    nodes.append(
        Node(
            package='autocarz',
            executable='roi_in',
            name='roi_in',
            namespace='vlp',
            output='screen',
            parameters=[
                {
                    'path_in': path_in,

                    'topics.pointcloud':
                        vlp_pointcloud_topic,
                    'topics.odom':
                        '/odom',
                    'topics.waypoint_info':
                        '/waypointInfo',

                    'topics.roi':
                        '/vlp/roi_in',
                    'topics.max_roi':
                        '/vlp/max_roi_dis_in',

                    'roi_idxlen': 54,
                    'roi_idx_start': 7,
                    'is_livox': False,
                    'radius_dim': 0.1,
                }
            ],
        )
    )

    nodes.append(
        Node(
            package='autocarz',
            executable='roi_out',
            name='roi_out',
            namespace='vlp',
            output='screen',
            parameters=[
                {
                    'path_out': path_out,

                    'topics.pointcloud':
                        vlp_pointcloud_topic,
                    'topics.odom':
                        '/odom',
                    'topics.waypoint_info':
                        '/waypointInfo',

                    'topics.roi':
                        '/vlp/roi_out',
                    'topics.max_roi':
                        '/vlp/max_roi_dis_out',

                    'roi_idxlen': 54,
                    'roi_idx_start': 7,
                    'is_livox': False,
                    'radius_dim': 0.2,
                }
            ],
        )
    )

    nodes.append(
        Node(
            package='autocarz',
            executable='ransac_in',
            name='ransac_in',
            namespace='vlp',
            output='screen',
            parameters=[
                {
                    'z_threshold': 0.1,
                    'topics.input':
                        '/vlp/roi_in',
                    'topics.output':
                        '/vlp/ransac_in',
                }
            ],
        )
    )

    nodes.append(
        Node(
            package='autocarz',
            executable='ransac_out',
            name='ransac_out',
            namespace='vlp',
            output='screen',
            parameters=[
                {
                    'z_threshold': 0.1,
                    'topics.input':
                        '/vlp/roi_out',
                    'topics.output':
                        '/vlp/ransac_out',
                }
            ],
        )
    )

    nodes.append(
        Node(
            package='autocarz',
            executable='cluster_in',
            name='cluster_in',
            namespace='vlp',
            output='screen',
            parameters=[
                {
                    'topics.input':
                        '/vlp/ransac_in',
                    'topics.waypoint_info':
                        '/waypointInfo',

                    'topics.distance_front':
                        '/vlp/distance_front',
                    'topics.distance_side_front':
                        '/vlp/distance_side_front',
                    'topics.distance_side_back':
                        '/vlp/distance_side_back',

                    'topics.cluster':
                        '/vlp/cluster_in',
                }
            ],
        )
    )

    nodes.append(
        Node(
            package='autocarz',
            executable='cluster_out',
            name='cluster_out',
            namespace='vlp',
            output='screen',
            parameters=[
                {
                    'topics.input':
                        '/vlp/ransac_out',
                    'topics.waypoint_info':
                        '/waypointInfo',

                    'topics.distance_front':
                        '/vlp/distance_front',
                    'topics.distance_side_front':
                        '/vlp/distance_side_front',
                    'topics.distance_side_back':
                        '/vlp/distance_side_back',

                    'topics.cluster':
                        '/vlp/cluster_out',
                }
            ],
        )
    )

    # =========================================================================
    # Livox 파이프라인
    #
    # /livox/lidar
    #   ├─ /livox/roi_in
    #   │    └─ /livox/ransac_in
    #   │          └─ /livox/cluster_in
    #   └─ /livox/roi_out
    #        └─ /livox/ransac_out
    #              └─ /livox/cluster_out
    # =========================================================================

    nodes.append(
        Node(
            package='autocarz',
            executable='roi_in',
            name='roi_in',
            namespace='livox',
            output='screen',
            parameters=[
                {
                    'path_in': path_in,

                    'topics.pointcloud':
                        livox_pointcloud_topic,
                    'topics.odom':
                        '/odom',
                    'topics.waypoint_info':
                        '/waypointInfo',

                    'topics.roi':
                        '/livox/roi_in',
                    'topics.max_roi':
                        '/livox/max_roi_dis_in',

                    'roi_idxlen': 100,
                    'roi_idx_start': 7,
                    'is_livox': True,
                    'radius_dim': 0.2,
                }
            ],
        )
    )

    nodes.append(
        Node(
            package='autocarz',
            executable='roi_out',
            name='roi_out',
            namespace='livox',
            output='screen',
            parameters=[
                {
                    'path_out': path_out,

                    'topics.pointcloud':
                        livox_pointcloud_topic,
                    'topics.odom':
                        '/odom',
                    'topics.waypoint_info':
                        '/waypointInfo',

                    'topics.roi':
                        '/livox/roi_out',
                    'topics.max_roi':
                        '/livox/max_roi_dis_out',

                    'roi_idxlen': 100,
                    'roi_idx_start': 7,
                    'is_livox': True,
                    'radius_dim': 0.3,
                }
            ],
        )
    )

    nodes.append(
        Node(
            package='autocarz',
            executable='ransac_in',
            name='ransac_in',
            namespace='livox',
            output='screen',
            parameters=[
                {
                    'z_threshold': 0.25,
                    'topics.input':
                        '/livox/roi_in',
                    'topics.output':
                        '/livox/ransac_in',
                }
            ],
        )
    )

    nodes.append(
        Node(
            package='autocarz',
            executable='ransac_out',
            name='ransac_out',
            namespace='livox',
            output='screen',
            parameters=[
                {
                    'z_threshold': 0.25,
                    'topics.input':
                        '/livox/roi_out',
                    'topics.output':
                        '/livox/ransac_out',
                }
            ],
        )
    )

    nodes.append(
        Node(
            package='autocarz',
            executable='cluster_in',
            name='cluster_in',
            namespace='livox',
            output='screen',
            parameters=[
                {
                    'topics.input':
                        '/livox/ransac_in',
                    'topics.waypoint_info':
                        '/waypointInfo',

                    'topics.distance_front':
                        '/livox/distance_front',
                    'topics.distance_side_front':
                        '/livox/distance_side_front',
                    'topics.distance_side_back':
                        '/livox/distance_side_back',

                    'topics.cluster':
                        '/livox/cluster_in',
                }
            ],
        )
    )

    nodes.append(
        Node(
            package='autocarz',
            executable='cluster_out',
            name='cluster_out',
            namespace='livox',
            output='screen',
            parameters=[
                {
                    'topics.input':
                        '/livox/ransac_out',
                    'topics.waypoint_info':
                        '/waypointInfo',

                    'topics.distance_front':
                        '/livox/distance_front',
                    'topics.distance_side_front':
                        '/livox/distance_side_front',
                    'topics.distance_side_back':
                        '/livox/distance_side_back',

                    'topics.cluster':
                        '/livox/cluster_out',
                }
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
                    'use Carla PointCloud2 topic'
                ),
            ),

            OpaqueFunction(
                function=launch_setup
            ),
        ]
    )
