#!/usr/bin/env python3

import math
from typing import Optional

from ackermann_msgs.msg import AckermannDriveStamped
from autocarz.msg import CtrlCmd, Erp42State
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus, PointCloud2, PointField


CONTROL_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)

SENSOR_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)


def clamp(value, min_value, max_value):
    return max(min_value, min(max_value, value))


def field_offset(msg, name):
    for field in msg.fields:
        if field.name == name:
            return field.offset if field.datatype == PointField.FLOAT32 else None
    return None


def ensure_xyzi_cloud(msg):
    if any(field.name == 'intensity' for field in msg.fields):
        return msg

    x_offset = field_offset(msg, 'x')
    y_offset = field_offset(msg, 'y')
    z_offset = field_offset(msg, 'z')
    if x_offset is None or y_offset is None or z_offset is None:
        return msg

    import struct

    float_fmt = '>f' if msg.is_bigendian else '<f'
    unpack_float = struct.Struct(float_fmt).unpack_from
    pack_float = struct.Struct('<f').pack_into
    point_count = msg.width * msg.height
    point_step = 16
    data = bytearray(point_count * point_step)

    for row in range(msg.height):
        for col in range(msg.width):
            src = row * msg.row_step + col * msg.point_step
            dst = (row * msg.width + col) * point_step
            pack_float(data, dst, unpack_float(msg.data, src + x_offset)[0])
            pack_float(data, dst + 4, unpack_float(msg.data, src + y_offset)[0])
            pack_float(data, dst + 8, unpack_float(msg.data, src + z_offset)[0])
            pack_float(data, dst + 12, 0.0)

    cloud = PointCloud2()
    cloud.header = msg.header
    cloud.height = msg.height
    cloud.width = msg.width
    cloud.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    cloud.is_bigendian = False
    cloud.point_step = point_step
    cloud.row_step = msg.width * point_step
    cloud.data = bytes(data)
    cloud.is_dense = msg.is_dense
    return cloud


class DfsCoordinateConverter:
    """Inverse of autocarz sensor_data_isro.latlong2xy()."""

    def __init__(self):
        self.re = 6371.00877 / 5e-07
        self.slat1 = math.radians(30.0)
        self.slat2 = math.radians(60.0)
        self.olon = math.radians(126.0)
        self.olat = math.radians(38.0)
        self.xo = 43.0
        self.yo = 136.0
        self.first_xx = -29180.0
        self.first_yy = 511660.0

        self.sn = math.tan(math.pi * 0.25 + self.slat2 * 0.5) / math.tan(math.pi * 0.25 + self.slat1 * 0.5)
        self.sn = math.log(math.cos(self.slat1) / math.cos(self.slat2)) / math.log(self.sn)
        self.sf = math.tan(math.pi * 0.25 + self.slat1 * 0.5)
        self.sf = (self.sf ** self.sn) * math.cos(self.slat1) / self.sn
        ro = math.tan(math.pi * 0.25 + self.olat * 0.5)
        self.ro = self.re * self.sf / (ro ** self.sn)

    def path_to_latlon(self, path_x, path_y):
        grid_x = (float(path_x) - self.first_xx) * 2000.0
        grid_y = (float(path_y) - self.first_yy) * 2000.0

        xn = grid_x - self.xo
        yn = self.ro - grid_y + self.yo
        ra = math.hypot(xn, yn)
        if self.sn < 0.0:
            ra = -ra

        alat = 2.0 * math.atan((self.re * self.sf / ra) ** (1.0 / self.sn)) - math.pi * 0.5
        theta = math.atan2(xn, yn)
        alon = theta / self.sn + self.olon
        return math.degrees(alat), math.degrees(alon)


class CarlaRealTopicAdapter(Node):
    def __init__(self):
        super().__init__('carla_real_topic_adapter')

        self.declare_parameter('carla_odom_topic', '/carla/ego_vehicle/odom')
        self.declare_parameter('ctrl_topic', '/ctrlCmd')
        self.declare_parameter('carla_control_topic', '/carla/ego_vehicle/ackermann_control_cmd')
        self.declare_parameter('vlp_carla_topic', '/carla/ego_vehicle/lidar_vlp16/point_cloud')
        self.declare_parameter('livox_carla_topic', '/carla/ego_vehicle/lidar_livox/point_cloud')
        self.declare_parameter('fix_topic', '/fix')
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('erp42_state_topic', '/erp42/state')
        self.declare_parameter('velodyne_topic', '/velodyne_points')
        self.declare_parameter('livox_topic', '/livox/lidar')
        self.declare_parameter('speed_scale', 1.0 / 3.6)
        self.declare_parameter('steering_scale', -1.0)
        self.declare_parameter('max_speed_mps', 3.0)
        self.declare_parameter('max_steering_rad', 0.6)
        self.declare_parameter('acceleration_mps2', 1.0)
        self.declare_parameter('brake_deceleration_mps2', 4.0)
        self.declare_parameter('publish_lidar_relays', True)
        self.declare_parameter('publish_erp42_state', True)

        self.converter = DfsCoordinateConverter()
        self.last_odom: Optional[Odometry] = None

        self.speed_scale = float(self.get_parameter('speed_scale').value)
        self.steering_scale = float(self.get_parameter('steering_scale').value)
        self.max_speed_mps = max(0.1, float(self.get_parameter('max_speed_mps').value))
        self.max_steering_rad = max(0.01, float(self.get_parameter('max_steering_rad').value))
        self.acceleration_mps2 = max(0.1, float(self.get_parameter('acceleration_mps2').value))
        self.brake_deceleration_mps2 = -max(0.1, float(self.get_parameter('brake_deceleration_mps2').value))

        self.fix_pub = self.create_publisher(NavSatFix, str(self.get_parameter('fix_topic').value), CONTROL_QOS)
        self.imu_pub = self.create_publisher(Imu, str(self.get_parameter('imu_topic').value), CONTROL_QOS)
        self.publish_erp42_state = bool(self.get_parameter('publish_erp42_state').value)
        self.erp42_pub = None
        if self.publish_erp42_state:
            self.erp42_pub = self.create_publisher(
                Erp42State,
                str(self.get_parameter('erp42_state_topic').value),
                CONTROL_QOS,
            )
        self.ackermann_pub = self.create_publisher(
            AckermannDriveStamped,
            str(self.get_parameter('carla_control_topic').value),
            CONTROL_QOS,
        )

        self.create_subscription(
            Odometry,
            str(self.get_parameter('carla_odom_topic').value),
            self.odom_callback,
            SENSOR_QOS,
        )
        self.create_subscription(
            CtrlCmd,
            str(self.get_parameter('ctrl_topic').value),
            self.ctrl_callback,
            CONTROL_QOS,
        )

        if bool(self.get_parameter('publish_lidar_relays').value):
            self.velodyne_pub = self.create_publisher(
                PointCloud2,
                str(self.get_parameter('velodyne_topic').value),
                SENSOR_QOS,
            )
            self.livox_pub = self.create_publisher(
                PointCloud2,
                str(self.get_parameter('livox_topic').value),
                SENSOR_QOS,
            )
            self.create_subscription(
                PointCloud2,
                str(self.get_parameter('vlp_carla_topic').value),
                self.vlp_callback,
                SENSOR_QOS,
            )
            self.create_subscription(
                PointCloud2,
                str(self.get_parameter('livox_carla_topic').value),
                self.livox_callback,
                SENSOR_QOS,
            )

        self.create_timer(1.0, self.status_timer)
        self.get_logger().info('CARLA real-topic adapter started without modifying autocarz.')

    def odom_callback(self, msg):
        self.last_odom = msg
        stamp = self.get_clock().now().to_msg()
        path_x = msg.pose.pose.position.x
        path_y = msg.pose.pose.position.y
        lat, lon = self.converter.path_to_latlon(path_x, path_y)

        fix = NavSatFix()
        fix.header.stamp = stamp
        fix.header.frame_id = 'gps'
        fix.status.status = NavSatStatus.STATUS_GBAS_FIX
        fix.status.service = NavSatStatus.SERVICE_GPS
        fix.latitude = lat
        fix.longitude = lon
        fix.altitude = msg.pose.pose.position.z
        fix.position_covariance = [0.0] * 9
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_KNOWN
        self.fix_pub.publish(fix)

        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = 'imu'
        imu.orientation = msg.pose.pose.orientation
        imu.orientation_covariance = [0.0] * 9
        self.imu_pub.publish(imu)

        if self.erp42_pub is not None:
            state = Erp42State()
            state.velocity = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y) * 3.6
            state.steering = 0.0
            self.erp42_pub.publish(state)

    def ctrl_callback(self, msg):
        target_speed_mps = clamp(float(msg.velocity) * self.speed_scale, 0.0, self.max_speed_mps)
        steering_rad = clamp(float(msg.steering) * self.steering_scale, -self.max_steering_rad, self.max_steering_rad)
        brake_cmd = clamp(float(msg.brake) / 100.0, 0.0, 1.0)

        ackermann = AckermannDriveStamped()
        ackermann.header.stamp = self.get_clock().now().to_msg()
        ackermann.header.frame_id = 'base_link'
        ackermann.drive.speed = target_speed_mps
        ackermann.drive.steering_angle = steering_rad
        ackermann.drive.acceleration = self.brake_deceleration_mps2 * brake_cmd if brake_cmd > 0.0 else self.acceleration_mps2
        ackermann.drive.jerk = 0.0
        self.ackermann_pub.publish(ackermann)

    def vlp_callback(self, msg):
        cloud = ensure_xyzi_cloud(msg)
        cloud.header.frame_id = 'velodyne'
        self.velodyne_pub.publish(cloud)

    def livox_callback(self, msg):
        cloud = ensure_xyzi_cloud(msg)
        cloud.header.frame_id = 'lidar_livox'
        self.livox_pub.publish(cloud)

    def status_timer(self):
        if self.last_odom is None:
            self.get_logger().warning('Waiting for CARLA odom topic.')


def main(args=None):
    rclpy.init(args=args)
    node = CarlaRealTopicAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
