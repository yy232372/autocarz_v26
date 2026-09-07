#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import re as _re
from typing import Any as _Any
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy, qos_profile_sensor_data

CONTROL_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)
_MISSING = object()

def _normalize_parameter_name(name: str) -> str:
    value = str(name).strip()
    if value.startswith('~'):
        value = value[1:]
    value = value.strip('/').replace('/', '.')
    value = _re.sub(r'[^A-Za-z0-9_.]', '_', value)
    value = _re.sub(r'\.+', '.', value).strip('.')
    return value or 'unnamed'

def _get_parameter(node: Node, name: str, default: _Any = _MISSING):
    parameter_name = _normalize_parameter_name(name)
    if not node.has_parameter(parameter_name):
        if default is _MISSING:
            raise KeyError(f"필수 ROS 2 파라미터 '{parameter_name}'가 없습니다.")
        node.declare_parameter(parameter_name, default)
    return node.get_parameter(parameter_name).value

def _set_parameter(node: Node, name: str, value: _Any) -> None:
    from rclpy.parameter import Parameter
    parameter_name = _normalize_parameter_name(name)
    if not node.has_parameter(parameter_name):
        node.declare_parameter(parameter_name, value)
    else:
        node.set_parameters([Parameter(parameter_name, value=value)])

def _topic(node: Node, default_topic: str) -> str:
    slug = default_topic.strip('/')
    slug = _re.sub(r'[^A-Za-z0-9_]', '_', slug)
    slug = _re.sub(r'_+', '_', slug).strip('_') or 'root'
    parameter_name = f'topics.{slug}'
    if not node.has_parameter(parameter_name):
        node.declare_parameter(parameter_name, default_topic)
    return str(node.get_parameter(parameter_name).value)

from sensor_msgs.msg import NavSatFix, Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, UInt32
from autocarz.msg import Erp42State
from math import atan2, sin, tan, cos, log, floor, pi

class LidarSensor:
    """
    Handles Lidar sensor data.
    """

    def __init__(self, node: Node):
        self.node = node
        self._subscriptions = []
        self._subscriptions.append(self.node.create_subscription(Float32, _topic(self.node, 'vlp/distance_front'), self.get_vlp_front, CONTROL_QOS))
        self._subscriptions.append(self.node.create_subscription(Float32, _topic(self.node, 'vlp/distance_side_front'), self.get_vlp_side_front, CONTROL_QOS))
        self._subscriptions.append(self.node.create_subscription(Float32, _topic(self.node, 'vlp/distance_side_back'), self.get_vlp_side_back, CONTROL_QOS))
        self._subscriptions.append(self.node.create_subscription(Float32, _topic(self.node, 'livox/distance_front'), self.get_livox_front, CONTROL_QOS))
        self._subscriptions.append(self.node.create_subscription(Float32, _topic(self.node, 'livox/distance_side_front'), self.get_livox_side_front, CONTROL_QOS))
        self._subscriptions.append(self.node.create_subscription(Float32, _topic(self.node, 'livox/distance_side_back'), self.get_livox_side_back, CONTROL_QOS))
        self._subscriptions.append(self.node.create_subscription(Float32, _topic(self.node, 'livox/max_roi_dis_in'), self.get_livox_roi_in, CONTROL_QOS))
        self._subscriptions.append(self.node.create_subscription(Float32, _topic(self.node, 'livox/max_roi_dis_out'), self.get_livox_roi_out, CONTROL_QOS))
        self.vlp_front = 30.0
        self.vlp_side_front = 30.0
        self.vlp_side_back = -15.0
        self.livox_front = 30.0
        self.livox_side_front = 30.0
        self.livox_side_back = -15.0
        self.livox_roi_in = 30.0
        self.livox_roi_out = 30.0
        self.VLP_DISTANCE = 15.0

    def get_vlp_front(self, data):
        self.vlp_front = data.data

    def get_vlp_side_front(self, data):
        self.vlp_side_front = data.data

    def get_vlp_side_back(self, data):
        self.vlp_side_back = data.data

    def get_livox_front(self, data):
        self.livox_front = data.data + 0.6

    def get_livox_side_front(self, data):
        self.livox_side_front = data.data + 0.6

    def get_livox_side_back(self, data):
        self.livox_side_back = data.data

    def get_livox_roi_in(self, data):
        self.livox_roi_in = data.data + 0.6

    def get_livox_roi_out(self, data):
        self.livox_roi_out = data.data + 0.6

    def lidar_data(self, lane):
        """
        livox, vlp 의 모든 토픽을 종합하여 아래 3가지 변수를 채움
        lidar_d_front
        lidar_d_side_front
        lidar_d_side_back
        """
        if lane == 1:
            max_roi_dis_front = self.livox_roi_in
            max_roi_dis_side = self.livox_roi_out
        else:
            max_roi_dis_front = self.livox_roi_out
            max_roi_dis_side = self.livox_roi_in
        if max_roi_dis_front < self.livox_front:
            if self.vlp_front <= self.VLP_DISTANCE:
                lidar_d_front = self.vlp_front
            else:
                lidar_d_front = max(self.VLP_DISTANCE, max_roi_dis_front)
        else:
            lidar_d_front = min(self.vlp_front, self.livox_front)
        if max_roi_dis_side < self.livox_side_front:
            if self.vlp_side_front <= self.VLP_DISTANCE:
                lidar_d_side_front = self.vlp_side_front
            elif max_roi_dis_side < self.VLP_DISTANCE:
                lidar_d_side_front = self.VLP_DISTANCE
            else:
                lidar_d_side_front = max_roi_dis_side
        else:
            lidar_d_side_front = min(self.vlp_side_front, self.livox_side_front)
        lidar_d_side_back = self.vlp_side_back
        return (lidar_d_front, lidar_d_side_front, lidar_d_side_back)

class ISRO:
    """
    Handles ISRO GPS and IMU data.
    """

    def __init__(self, node: Node):
        self.node = node
        self._subscriptions = []
        self.map_offset_x = float(
            _get_parameter(self.node, 'coordinates.map_offset_x', -125320.0)
        )
        self.map_offset_y = float(
            _get_parameter(self.node, 'coordinates.map_offset_y', 136746.0)
        )
        self._subscriptions.append(self.node.create_subscription(NavSatFix, _topic(self.node, '/fix'), self.getGPS, qos_profile_sensor_data))
        self._subscriptions.append(self.node.create_subscription(Imu, _topic(self.node, '/imu/data'), self.getIMU, qos_profile_sensor_data))
        self._subscriptions.append(self.node.create_subscription(UInt32, _topic(self.node, '/pva/position_type'), self.get_position_type, qos_profile_sensor_data))
        self._subscriptions.append(self.node.create_subscription(UInt32, _topic(self.node, '/pva/ins_status'), self.get_ins_status, qos_profile_sensor_data))
        self._subscriptions.append(self.node.create_subscription(UInt32, _topic(self.node, '/pva/alignment_state'), self.get_alignment_state, qos_profile_sensor_data))
        self._subscriptions.append(self.node.create_subscription(UInt32, _topic(self.node, '/pva/time_since_update'), self.get_time_since_update, qos_profile_sensor_data))
        self.odomPub = self.node.create_publisher(Odometry, _topic(self.node, '/odom'), CONTROL_QOS)
        self.odom = Odometry()
        self.gpsX = 0.0
        self.gpsY = 0.0
        self.orient_q = None
        self.heading = 0.0
        self.gps_status = 0
        self.gps_covariance = (0.0, 0.0, 0.0)
        self.imu_covariance = (0.0, 0.0, 0.0)
        self.position_type = 0
        self.ins_status = 0
        self.alignment_state = 0
        self.time_since_update = 0
        self.checkGPS = False
        self.checkIMU = False
        self.checkHeading = False
        self.checkPositionType = False
        self.checkINSStatus = False
        self.checkAlignmentState = False
        self.checkTimeSinceUpdate = False
        self.last_gps_in = self.node.get_clock().now()
        self.last_imu_in = self.node.get_clock().now()
        self.last_pva_status_in = self.node.get_clock().now()

    def normalize_angle(self, angle):
        while angle > pi:
            angle -= 2.0 * pi
        while angle < -pi:
            angle += 2.0 * pi
        return angle

    def getGPS(self, data):
        self.gpsX, self.gpsY = self.latlong2xy(data.latitude, data.longitude)
        self.last_gps_in = self.node.get_clock().now()
        self.gps_status = data.status.status
        self.gps_covariance = (
            data.position_covariance[0],
            data.position_covariance[4],
            data.position_covariance[8],
        )
        self.checkGPS = True

    def getIMU(self, data):
        if data.orientation_covariance[0] < 0.0:
            self.checkIMU = False
            self.checkHeading = False
            self.last_imu_in = self.node.get_clock().now()
            self.imu_covariance = (float('inf'), float('inf'), float('inf'))
            return
        self.orient_q = data.orientation
        yaw = atan2(
            2.0 * (self.orient_q.w * self.orient_q.z + self.orient_q.x * self.orient_q.y),
            1.0 - 2.0 * (self.orient_q.y * self.orient_q.y + self.orient_q.z * self.orient_q.z),
        )
        self.heading = self.normalize_angle(yaw)
        self.checkHeading = True
        self.last_imu_in = self.node.get_clock().now()
        self.imu_covariance = (
            data.orientation_covariance[0],
            data.orientation_covariance[4],
            data.orientation_covariance[8],
        )
        self.checkIMU = True

    def get_position_type(self, data):
        self.position_type = int(data.data)
        self.checkPositionType = True
        self.last_pva_status_in = self.node.get_clock().now()

    def get_ins_status(self, data):
        self.ins_status = int(data.data)
        self.checkINSStatus = True
        self.last_pva_status_in = self.node.get_clock().now()

    def get_alignment_state(self, data):
        self.alignment_state = int(data.data)
        self.checkAlignmentState = True
        self.last_pva_status_in = self.node.get_clock().now()

    def get_time_since_update(self, data):
        self.time_since_update = int(data.data)
        self.checkTimeSinceUpdate = True
        self.last_pva_status_in = self.node.get_clock().now()

    def make_odom(self):
        self.odom.header.stamp = self.node.get_clock().now().to_msg()
        self.odom.header.frame_id = 'map'
        self.odom.child_frame_id = 'base_link'
        if self.checkGPS:
            self.odom.pose.pose.position.x = self.gpsX
            self.odom.pose.pose.position.y = self.gpsY
            self.odom.pose.pose.position.z = 0.0
        if self.checkIMU and self.orient_q is not None:
            self.odom.pose.pose.orientation = self.orient_q
        gps_imu_dt = abs((self.last_gps_in - self.last_imu_in).nanoseconds) * 1e-9
        if self.checkGPS and self.checkIMU and gps_imu_dt <= 0.2:
            self.odomPub.publish(self.odom)

    def latlong2xy(self, lat, long):
        RE = 6371.00877
        GRID = 5e-07
        SLAT1 = 30.0
        SLAT2 = 60.0
        OLON = 126.0
        OLAT = 38.0
        XO = 43
        YO = 136
        DEGRAD = pi / 180.0
        re = RE / GRID
        slat1 = SLAT1 * DEGRAD
        slat2 = SLAT2 * DEGRAD
        olon = OLON * DEGRAD
        olat = OLAT * DEGRAD
        sn = tan(pi * 0.25 + slat2 * 0.5) / tan(pi * 0.25 + slat1 * 0.5)
        sn = log(cos(slat1) / cos(slat2)) / log(sn)
        sf = tan(pi * 0.25 + slat1 * 0.5)
        sf = pow(sf, sn) * cos(slat1) / sn
        ro = tan(pi * 0.25 + olat * 0.5)
        ro = re * sf / pow(ro, sn)
        ra = tan(pi * 0.25 + lat * DEGRAD * 0.5)
        ra = re * sf / pow(ra, sn)
        theta = long * DEGRAD - olon
        if theta > pi:
            theta -= 2.0 * pi
        if theta < -pi:
            theta += 2.0 * pi
        theta *= sn
        rs_x = floor(ra * sin(theta) + XO + 0.5)
        rs_y = floor(ro - ra * cos(theta) + YO + 0.5)
        # 기존 PIM222A 코드에는 다른 시험장(Jeju) 오프셋이 남아 있었다.
        # first_xx = -29180
        # first_yy = 511660
        first_xx = self.map_offset_x
        first_yy = self.map_offset_y
        rs_x = rs_x / 2000 + first_xx
        rs_y = rs_y / 2000 + first_yy
        return (rs_x, rs_y)

    def gps_data(self):
        return (self.gps_status, self.gps_covariance, self.checkGPS, self.last_gps_in)

    def imu_data(self):
        return (self.odom, self.heading, self.checkHeading, self.imu_covariance, self.checkIMU, self.last_imu_in)

    def pva_status_data(self):
        status_complete = (
            self.checkPositionType
            and self.checkINSStatus
            and self.checkAlignmentState
            and self.checkTimeSinceUpdate
        )
        return (
            self.position_type,
            self.ins_status,
            self.alignment_state,
            self.time_since_update,
            status_complete,
            self.last_pva_status_in,
        )

class VehicleState:
    """
    Handles vehicle state data like velocity and steering.
    """

    def __init__(self, node: Node):
        self.node = node
        self._subscriptions = []
        self._subscriptions.append(self.node.create_subscription(Erp42State, _topic(self.node, '/erp42/state'), self.get_vehicle_state, CONTROL_QOS))
        self.velocity = 0.0
        self.steering = 0.0
        self.checkState = False

    def get_vehicle_state(self, data):
        velocity = data.velocity
        if velocity > 100 or velocity < 0:
            self.velocity = 0.0
        else:
            self.velocity = velocity
        self.steering = data.steering
        self.checkState = True

    def vehicle_data(self):
        return (self.velocity, self.steering, self.checkState)
