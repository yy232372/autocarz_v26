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

import csv
from math import sqrt, sin, cos, atan2, acos, pi
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from autocarz.msg import CtrlCmd, WaypointInfo
from geometry_msgs.msg import PoseStamped

class PathPlanner:
    """
    Handles path planning for the autonomous vehicle.
    """

    def __init__(self, node: Node, globalPathIn, globalPathOut):
        self.node = node
        self._subscriptions = []
        self.curWaypointIndxIn = 0
        self.curWaypointIndxOut = 0
        self.globalPathIn = globalPathIn
        self.globalPathOut = globalPathOut
        self.waypointInfo = WaypointInfo()
        self.waypointInfoPub = self.node.create_publisher(WaypointInfo, _topic(self.node, '/waypointInfo'), CONTROL_QOS)
        self.distance_log_in = self.node.create_publisher(Float32, _topic(self.node, 'distance_error_in'), CONTROL_QOS)
        self.distance_log_out = self.node.create_publisher(Float32, _topic(self.node, 'distance_error_out'), CONTROL_QOS)

    def find_nearest_waypoint(self, path, curX, curY):
        minDist = float('inf')
        nearest_idx = 0
        for i, pose in enumerate(path.poses):
            dx = curX - pose.pose.position.x
            dy = curY - pose.pose.position.y
            dist = sqrt(dx * dx + dy * dy)
            if dist < minDist:
                nearest_idx = i
                minDist = dist
        return (nearest_idx, minDist)

    def localization(self, odom, lane):
        """
        Publish distance errors for debugging and monitoring.
        """
        self.waypointInfo.lane = lane
        curX = odom.pose.pose.position.x
        curY = odom.pose.pose.position.y
        curWaypointIndxIn, minDistIn = self.find_nearest_waypoint(self.globalPathIn, curX, curY)
        curWaypointIndxOut, minDistOut = self.find_nearest_waypoint(self.globalPathOut, curX, curY)
        temp1 = Float32()
        temp2 = Float32()
        temp1.data = minDistIn
        temp2.data = minDistOut
        self.distance_log_in.publish(temp1)
        self.distance_log_out.publish(temp2)
        self.waypointInfo.indx_in = curWaypointIndxIn
        self.waypointInfo.indx_out = curWaypointIndxOut
        self.waypointInfoPub.publish(self.waypointInfo)
        return (curWaypointIndxIn, curWaypointIndxOut, minDistIn, minDistOut)

    def find_nearest_waypoint_stanley(self, path, curX, curY):
        minDist = float('inf')
        path_len = len(path.poses)
        nearest_idx = 0
        lfd = 10

        def normalize_angle(angle):
            while angle > pi:
                angle -= 2 * pi
            while angle < -pi:
                angle += 2 * pi
            return angle
        for i, pose in enumerate(path.poses):
            dx = curX - pose.pose.position.x
            dy = curY - pose.pose.position.y
            dist = sqrt(dx * dx + dy * dy)
            if dist < minDist:
                mdx = dx
                mdy = dy
                yaw_x = path.poses[(i + lfd) % path_len].pose.position.x - path.poses[i].pose.position.x
                yaw_y = path.poses[(i + lfd) % path_len].pose.position.y - path.poses[i].pose.position.y
                nearest_idx = i
                minDist = dist
        yaw = atan2(yaw_y, yaw_x)
        yaw_adjusted = yaw + pi / 2
        yaw_adjusted = normalize_angle(yaw_adjusted)
        return (nearest_idx, minDist, (mdx, mdy, yaw))

    def localization_pp_stanley(self, odom, odom_front, lane, stanley_flag):
        """
        Publish distance errors for debugging and monitoring.
        """
        self.waypointInfo.lane = lane
        if stanley_flag:
            curX = odom_front.pose.pose.position.x
            curY = odom_front.pose.pose.position.y
            curWaypointIndxIn, minDistIn, stanley_param_in = self.find_nearest_waypoint_stanley(self.globalPathIn, curX, curY)
            curWaypointIndxOut, minDistOut, stanley_param_out = self.find_nearest_waypoint_stanley(self.globalPathOut, curX, curY)
            temp1 = Float32()
            temp2 = Float32()
            temp1.data = minDistIn
            temp2.data = minDistOut
            self.distance_log_in.publish(temp1)
            self.distance_log_out.publish(temp2)
            self.waypointInfo.indx_in = curWaypointIndxIn
            self.waypointInfo.indx_out = curWaypointIndxOut
            self.waypointInfoPub.publish(self.waypointInfo)
            return (curWaypointIndxIn, curWaypointIndxOut, minDistIn, minDistOut, stanley_param_in, stanley_param_out)
        else:
            curX = odom.pose.pose.position.x
            curY = odom.pose.pose.position.y
            curWaypointIndxIn, minDistIn, stanley_param_in = self.find_nearest_waypoint(self.globalPathIn, curX, curY)
            curWaypointIndxOut, minDistOut, stanley_param_out = self.find_nearest_waypoint(self.globalPathOut, curX, curY)
            temp1 = Float32()
            temp2 = Float32()
            temp1.data = minDistIn
            temp2.data = minDistOut
            self.distance_log_in.publish(temp1)
            self.distance_log_out.publish(temp2)
            self.waypointInfo.indx_in = curWaypointIndxIn
            self.waypointInfo.indx_out = curWaypointIndxOut
            self.waypointInfoPub.publish(self.waypointInfo)
            return (curWaypointIndxIn, curWaypointIndxOut, minDistIn, minDistOut, stanley_param_in, stanley_param_out)

class SpeedDecision:
    """
    Manages the speed of the vehicle based on various factors.
    """

    def __init__(self, node: Node):
        self.node = node
        self._subscriptions = []
        self.ctrlPub = self.node.create_publisher(CtrlCmd, _topic(self.node, '/ctrlCmd'), CONTROL_QOS)
        self.ctrl_msg = CtrlCmd()

    def velocity_scale_path_dict(self, path_idx, add_speed, ref_velocity):
        velocity_scale_map = {0: (18.0 + add_speed) / ref_velocity, 1: (16.0 + add_speed) / ref_velocity, 2: (16.0 + add_speed) / ref_velocity, 3: (20.0 + add_speed) / ref_velocity, 4: (20.0 + add_speed) / ref_velocity}
        return velocity_scale_map.get(path_idx, 'path_path_index_error')

    def control_speed(self, lidar_d_front, path_idx, velocity_scale_side):
        add_speed = 0.0
        ref_velocity = 18.0 + add_speed
        velocity_scale_path = self.velocity_scale_path_dict(path_idx, add_speed, ref_velocity)
        min_v = 6.0 * velocity_scale_side
        max_v = 18.0 * velocity_scale_side * velocity_scale_path
        min_d = 8.0
        max_d = 23.0
        criteria_d = (min_d + max_d) / 2.0
        criteria_v = (min_v + (max_v - min_v) * (criteria_d - min_d) / (max_d - min_d)) * velocity_scale_side * velocity_scale_path
        p_gain = (max_v - min_v) / (max_d - min_d)
        i_gain = 0.1
        d_gain = 0.0
        controlTime = 0.1
        prev_error = 0.0
        i_control = 0.0
        brake = 0
        if lidar_d_front < min_d:
            target_velocity = 0
            brake = 100
        else:
            target_velocity = max_v
        return (target_velocity, brake)

    def control_speed_curve(self, lidar_d_front, path_idx, velocity_scale_side):
        add_speed = 0.0
        ref_velocity = 16.0 + add_speed
        velocity_scale_path = self.velocity_scale_path_dict(path_idx, add_speed, ref_velocity)
        min_v = 6.0 * velocity_scale_side
        max_v = 16.0 * velocity_scale_side * velocity_scale_path
        min_d = 8.0
        max_d = 15.0
        criteria_d = (min_d + max_d) / 2.0
        criteria_v = (min_v + (max_v - min_v) * (criteria_d - min_d) / (max_d - min_d)) * velocity_scale_side * velocity_scale_path
        p_gain = (max_v - min_v) / (max_d - min_d)
        i_gain = 0.1
        d_gain = 0.0
        controlTime = 0.1
        prev_error = 0.0
        i_control = 0.0
        brake = 0
        if lidar_d_front < min_d:
            target_velocity = 0
            brake = 100
        else:
            target_velocity = max_v
        return (target_velocity, brake)

    def gps_status_checker(self, gps_status, gps_covariance, last_gps_in, path_path_index):
        if self.node.get_clock().now() - last_gps_in > Duration(seconds=0.5):
            self.node.get_logger().info('GPS TOPIC TIME ERROR')
            return False
        if gps_status != 2:
            self.node.get_logger().info('GPS STATUS ERROR STOP ')
            return False
        return True

    def isGPSerror(self, gps_status_back, gps_covariance_back, last_back_gps_in, gps_status_front, gps_covariance_front, last_front_gps_in, path_path_index):
        back_gps_ok = self.gps_status_checker(gps_status_back, gps_covariance_back, last_back_gps_in, path_path_index)
        front_gps_ok = self.gps_status_checker(gps_status_front, gps_covariance_front, last_front_gps_in, path_path_index)
        if not back_gps_ok and (not front_gps_ok):
            self.ctrl_msg.longl_cmd_type = 2
            self.ctrl_msg.velocity = 0.0
            self.ctrl_msg.brake = 100
            self.ctrlPub.publish(self.ctrl_msg)
            return 'both_error'
        elif not back_gps_ok:
            self.ctrl_msg.longl_cmd_type = 2
            self.ctrl_msg.velocity = 0.0
            self.ctrl_msg.brake = 100
            self.ctrlPub.publish(self.ctrl_msg)
            return 'back_error'
        elif not front_gps_ok:
            return 'front_error'
        else:
            return 'no_error'

class SteeringDecision:
    """
    Decision the steering angle of the vehicle.
    """

    def __init__(self, node: Node, numGlobalPathIn, numGloablPathOut, globalPathIn, globalPathOut):
        self.node = node
        self._subscriptions = []
        self.carLength = 1.3
        self.lfdPointidx = 1
        self.lfdPub = self.node.create_publisher(Odometry, _topic(self.node, '/lfd'), CONTROL_QOS)
        self.pathInfo = {1: [numGlobalPathIn, globalPathIn], 2: [numGloablPathOut, globalPathOut]}
        self.K_in, self.K_out = self.calculate_curvature()

    def calculate_curvature(self):
        K_in = self.process_path(self.pathInfo[1][1])
        K_out = self.process_path(self.pathInfo[2][1])
        return (K_in, K_out)

    def calculate_lfd(self, velocity, path_idx, lane, in_index, out_index):
        K = self.K_in[in_index] if lane == 1 else self.K_out[out_index]
        if K < 2.5:
            K = 0
        if lane == 1:
            if path_idx == 1:
                lfd = 5.5
            elif path_idx == 2:
                lfd = 6.5
            else:
                lfd = 8.0
        elif path_idx == 1:
            lfd = 5.5
        elif path_idx == 2:
            lfd = 6.5
        else:
            lfd = 8.0
        return max(lfd, 3)

    def process_path(self, path):
        x = [pose.pose.position.x for pose in path.poses]
        y = [pose.pose.position.y for pose in path.poses]
        L = [0.0] * len(x)
        for i in range(len(x)):
            i_next = (i + 1) % len(x)
            i_next_next = (i + 2) % len(x)
            v1 = [x[i_next] - x[i], y[i_next] - y[i]]
            v2 = [x[i_next_next] - x[i_next], y[i_next_next] - y[i_next]]
            v1_norm = sqrt(v1[0] ** 2 + v1[1] ** 2)
            v2_norm = sqrt(v2[0] ** 2 + v2[1] ** 2)
            if v1_norm > 1e-06:
                v1 = [v1[0] / v1_norm, v1[1] / v1_norm]
            else:
                v1 = [0, 0]
            if v2_norm > 1e-06:
                v2 = [v2[0] / v2_norm, v2[1] / v2_norm]
            else:
                v2 = [0, 0]
            dot_product = v1[0] * v2[0] + v1[1] * v2[1]
            angle_diff = acos(max(min(dot_product, 1.0), -1.0))
            L[i] = angle_diff * 180 / pi
        K = [0.0] * len(x)
        for i in range(len(x)):
            weights = [0.95 - 0.05 * j for j in range(20)]
            indices = [(i + j) % len(x) for j in range(20)]
            K[i] = sum((L[idx] * weights[j] for j, idx in enumerate(indices)))
        return K

    def purePursuit(self, odom, lane, velocity, heading, path_idx, curWaypointIndxIn, curWaypointIndxOut):
        curX = odom.pose.pose.position.x
        curY = odom.pose.pose.position.y
        bound = 150
        steering = 0
        lfd = self.calculate_lfd(velocity, path_idx, lane, curWaypointIndxIn, curWaypointIndxOut)
        if not lfd:
            return None
        curWaypoint = curWaypointIndxIn if lane == 1 else curWaypointIndxOut
        if curWaypoint + bound >= self.pathInfo[lane][0]:
            localWaypoints = range(curWaypoint, self.pathInfo[lane][0]) + range(0, bound - (self.pathInfo[lane][0] - curWaypoint))
        else:
            localWaypoints = range(curWaypoint, curWaypoint + bound)
        for i in localWaypoints:
            dx = self.pathInfo[lane][1].poses[i].pose.position.x - curX
            dy = self.pathInfo[lane][1].poses[i].pose.position.y - curY
            rx = cos(heading) * dx + sin(heading) * dy
            ry = sin(heading) * dx - cos(heading) * dy
            if rx > 0.0:
                dist = sqrt(rx * rx + ry * ry)
                if dist >= lfd:
                    theta = atan2(ry, rx)
                    steering = -atan2(2 * self.carLength * sin(theta), lfd)
                    self.lfdPointidx = i
                    break
        lfdPoint = Odometry()
        lfdPoint.header.frame_id = 'map'
        lfdPoint.pose.pose.position.z = 0.0
        lfdPoint.pose.pose.orientation.x = 0.0
        lfdPoint.pose.pose.orientation.y = 0.0
        lfdPoint.pose.pose.orientation.z = 0.0
        lfdPoint.pose.pose.orientation.w = 1.0
        lfdPoint.pose.pose.position.x = self.pathInfo[lane][1].poses[self.lfdPointidx].pose.position.x
        lfdPoint.pose.pose.position.y = self.pathInfo[lane][1].poses[self.lfdPointidx].pose.position.y
        self.lfdPub.publish(lfdPoint)
        return steering

class LineChanger:
    """
    Decision line change or not.
    """

    def __init__(self, node: Node):
        self.node = node
        self._subscriptions = []
        self.temp_velocity = 0
        self.lane_changing_flag = False
        self.stop_line_timer = self.node.get_clock().now()
        self.rotary_stop_line_timer = self.node.get_clock().now()
        self.line_change_flag = False
        self.rotary_line_change_flag = False

    def line_change_check(self, path_index, distance, target_velocity, lidar_front, lidar_side_front, lidar_side_back, cur_lane):
        is_change = False
        temp_velocity = target_velocity
        if self.lane_changing_flag:
            temp_velocity, is_done = self.lane_changing(target_velocity, distance)
            if is_done:
                self.lane_changing_flag = False
                print('line change done')
        if target_velocity < 13 and 25 > lidar_front > 5 and (lidar_side_front > lidar_front + 5) and (lidar_side_back <= -15):
            self.time_recorder()
            if self.line_change_flag and self.node.get_clock().now() - self.stop_line_timer > Duration(seconds=1.0):
                self.line_change(target_velocity)
                is_change = True
        else:
            self.line_change_init()
        if path_index == 0:
            if target_velocity < 13 and 25 > lidar_front > 5 and (lidar_side_front > lidar_front + 5) and (lidar_side_back <= -15):
                self.time_recorder()
                if self.line_change_flag and self.node.get_clock().now() - self.stop_line_timer > Duration(seconds=1.0):
                    self.line_change(target_velocity)
                    is_change = True
            else:
                self.line_change_init()
        return (is_change, temp_velocity, self.lane_changing_flag)

    def time_recorder(self):
        if self.line_change_flag == False:
            self.stop_line_timer = self.node.get_clock().now()
            self.line_change_flag = True

    def line_change(self, target_velocity):
        self.line_change_init()
        self.temp_velocity = target_velocity
        print('temp vel', self.temp_velocity)
        self.lane_changing_flag = True

    def lane_changing(self, target_velocity, distance):
        done_flag = False
        if target_velocity > self.temp_velocity:
            target_velocity = max(self.temp_velocity, 7)
        if distance < 0.3:
            done_flag = True
        return (target_velocity, done_flag)

    def line_change_init(self):
        self.line_change_flag = False

class MergeChecker:
    """
    If merge section or not
    """

    def __init__(self):
        self.velocity_scale_side = 1.0
        self.moving_checker_flag = False
        self.merge_car_checker_flag = False

    def merge_check(self, curr_path_idx, lidar_side_front, lidar_side_back):
        """
        병합의 경우 차선 변경은 고려하지 않음.
        mergeChecker의 경우 Control 함수 내부에서 속도 계획의 일부로 사용됨. 
        병합 전 (8~0m) 주행에 대한 판단. 시작지점은 반복적인 주행으로 판단. 
        """
        if curr_path_idx == 2:
            if 0 < lidar_side_front < 15 or 0 > lidar_side_back > -15:
                if self.merge_car_checker_flag == False:
                    self.merge(lidar_side_front, lidar_side_back)
            else:
                self.merge_init()
        return self.velocity_scale_side

    def merge_init(self):
        self.moving_checker_flag = False
        self.merge_car_checker_flag = False
        self.velocity_scale_side = 1.0

    def merge(self, lidar_side_front, lidar_side_back):
        if 0 < lidar_side_front < 3 or -3 < lidar_side_back < 0:
            self.velocity_scale_side = 0.5
        else:
            self.merge_init()
