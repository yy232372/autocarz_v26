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

from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from autocarz.msg import Erp42State
from math import sqrt, atan2, sin, tan, cos, log, floor, pi

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

class GPS:
    """
    Handles GPS data.
    """

    def __init__(self, node: Node):
        self.node = node
        self._subscriptions = []
        self._subscriptions.append(self.node.create_subscription(NavSatFix, _topic(self.node, '/ublox1/fix'), self.getGPSBack, qos_profile_sensor_data))
        self._subscriptions.append(self.node.create_subscription(NavSatFix, _topic(self.node, '/ublox2/fix'), self.getGPSFront, qos_profile_sensor_data))
        self.odomPub = self.node.create_publisher(Odometry, _topic(self.node, '/odom'), CONTROL_QOS)
        self.odom = Odometry()
        self.odomFront = Odometry()
        self.heading = 0.0
        self.gps_status = 0
        self.gps_statusFront = 0
        self.gps_covariance = 0.0
        self.gps_covarianceFront = 0.0
        self.pastPos = [0.0, 0.0]
        self.checkGPS = False
        self.checkGPSFront = False
        self.checkHeading = False
        self.last_gps_in = self.node.get_clock().now()
        self.last_gps_inFront = self.node.get_clock().now()

    def getGPSBack(self, data):
        gpsX, gpsY = self.latlong2xy(data.latitude, data.longitude)
        self.odom.header.frame_id = 'map'
        self.odom.pose.pose.position.x = gpsX
        self.odom.pose.pose.position.y = gpsY
        self.odom.pose.pose.position.z = 0.0
        self.last_gps_in = self.node.get_clock().now()
        self.gps_status = data.status.status
        self.gps_covariance = data.position_covariance[0]
        if self.checkGPS:
            if self.checkGPSFront and self.last_gps_in - self.last_gps_inFront <= Duration(seconds=0.11) and (self.gps_statusFront == 2 or self.gps_covarianceFront < 0.05):
                dx = self.odomFront.pose.pose.position.x - gpsX
                dy = self.odomFront.pose.pose.position.y - gpsY
                self.heading = atan2(dy, dx)
                self.odom.pose.pose.orientation.x = 0.0
                self.odom.pose.pose.orientation.y = 0.0
                self.odom.pose.pose.orientation.z = sin(self.heading * 0.5)
                self.odom.pose.pose.orientation.w = cos(self.heading * 0.5)
                self.checkGPS = False
                self.checkHeading = True
            else:
                dx = gpsX - self.pastPos[0]
                dy = gpsY - self.pastPos[1]
                dist = sqrt(dx ** 2 + dy ** 2)
                if dist > 0.1:
                    self.heading = atan2(dy, dx)
                    self.odom.pose.pose.orientation.x = 0.0
                    self.odom.pose.pose.orientation.y = 0.0
                    self.odom.pose.pose.orientation.z = sin(self.heading * 0.5)
                    self.odom.pose.pose.orientation.w = cos(self.heading * 0.5)
                    self.checkGPS = False
                    self.checkHeading = True
        if not self.checkGPS:
            self.pastPos = [gpsX, gpsY]
            self.checkGPS = True
        if not self.checkHeading:
            print('No heading')
        self.odomPub.publish(self.odom)

    def getGPSFront(self, data):
        gpsX, gpsY = self.latlong2xy(data.latitude, data.longitude)
        self.odomFront.header.frame_id = 'map'
        self.odomFront.pose.pose.position.x = gpsX
        self.odomFront.pose.pose.position.y = gpsY
        self.odomFront.pose.pose.position.z = 0.0
        self.last_gps_inFront = self.node.get_clock().now()
        self.gps_statusFront = data.status.status
        self.gps_covarianceFront = data.position_covariance[0]
        self.checkGPSFront = True

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
        first_xx = -107981.357 + 79799
        first_yy = 186833.4085 + 324987
        rs_x = rs_x / 2000 + first_xx
        rs_y = rs_y / 2000 + first_yy
        return (rs_x, rs_y)

    def gps_data_back(self):
        return (self.odom, self.heading, self.gps_status, self.gps_covariance, self.checkGPS, self.checkHeading, self.last_gps_in)

    def gps_data_front(self):
        return (self.odomFront, self.gps_statusFront, self.gps_covarianceFront, self.checkGPSFront, self.last_gps_inFront)

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
