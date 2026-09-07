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
from math import sqrt, sin, cos, atan2, acos, pi, isfinite
from std_msgs.msg import Float32, Bool, String
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
        self.gps_ready_pub = self.node.create_publisher(Bool, _topic(self.node, '/pim222a/gps_ready'), CONTROL_QOS)
        self.imu_ready_pub = self.node.create_publisher(Bool, _topic(self.node, '/pim222a/imu_ready'), CONTROL_QOS)
        self.control_ready_pub = self.node.create_publisher(Bool, _topic(self.node, '/pim222a/control_ready'), CONTROL_QOS)
        self.health_reason_pub = self.node.create_publisher(String, _topic(self.node, '/pim222a/health_reason'), CONTROL_QOS)
        self.ctrl_msg = CtrlCmd()
        self.last_imu_cov_ok_log = None

        # PIM222A는 50 Hz PVA와 100 Hz IMU를 출력한다. 아래 값은 모두
        # launch 파라미터로 조정 가능하며, 경로 번호와 무관한 센서 품질
        # 기준으로 사용한다.
        self.gps_timeout_sec = float(_get_parameter(node, 'pim222a.gps_timeout_sec', 0.3))
        self.imu_timeout_sec = float(_get_parameter(node, 'pim222a.imu_timeout_sec', 0.2))
        self.pva_status_timeout_sec = float(_get_parameter(node, 'pim222a.status_timeout_sec', 0.3))
        self.max_horizontal_covariance = float(_get_parameter(node, 'pim222a.max_horizontal_covariance', 0.005))
        self.max_yaw_covariance = float(_get_parameter(node, 'pim222a.max_yaw_covariance', 0.01))
        self.max_time_since_update = int(_get_parameter(node, 'pim222a.max_time_since_update', 2))
        self.allowed_position_types = {
            int(value) for value in _get_parameter(node, 'pim222a.allowed_position_types', [56])
        }
        self.allowed_ins_statuses = {
            int(value) for value in _get_parameter(node, 'pim222a.allowed_ins_statuses', [3, 7])
        }
        self.allowed_alignment_states = {
            int(value) for value in _get_parameter(node, 'pim222a.allowed_alignment_states', [2, 3])
        }
        self.bad_cycles_to_stop = max(1, int(_get_parameter(node, 'pim222a.bad_cycles_to_stop', 2)))
        self.good_cycles_to_resume = max(1, int(_get_parameter(node, 'pim222a.good_cycles_to_resume', 20)))

        self.gps_ready = False
        self.imu_ready = False
        self.gps_good_cycles = 0
        self.gps_bad_cycles = 0
        self.imu_good_cycles = 0
        self.imu_bad_cycles = 0
        self.gps_reason = 'GPS startup check'
        self.imu_reason = 'IMU startup check'
        self._last_health_log = {}
        self._last_health_message = {}

    def _log_health(self, key, message, warning=False):
        """상태가 바뀌거나 2초가 지난 경우에만 로그를 남긴다."""
        now = self.node.get_clock().now()
        last_time = self._last_health_log.get(key)
        changed = self._last_health_message.get(key) != message
        if changed or last_time is None or now - last_time > Duration(seconds=2.0):
            if warning:
                self.node.get_logger().warning(message)
            else:
                self.node.get_logger().info(message)
            self._last_health_log[key] = now
            self._last_health_message[key] = message

    def _publish_health_reason(self, reason):
        self.health_reason_pub.publish(String(data=str(reason)))

    def hold_stop(self, reason):
        """센서 상태가 불안정할 때 반복 호출해도 안전한 정지 명령."""
        self.ctrl_msg.longl_cmd_type = 2
        self.ctrl_msg.velocity = 0.0
        self.ctrl_msg.brake = 100.0
        self.ctrlPub.publish(self.ctrl_msg)
        self.control_ready_pub.publish(Bool(data=False))
        self._publish_health_reason(reason)

    def mark_control_ready(self):
        self.control_ready_pub.publish(Bool(data=True))
        self._publish_health_reason('READY')

    def velocity_scale_path_dict(self, path_idx, add_speed, ref_velocity):
        velocity_scale_map = {0: (18.0 + add_speed) / ref_velocity, 1: (16.0 + add_speed) / ref_velocity, 2: (16.0 + add_speed) / ref_velocity, 3: (20.0 + add_speed) / ref_velocity, 4: (20.0 + add_speed) / ref_velocity}
        return velocity_scale_map.get(path_idx, 'path_path_index_error')

    def control_speed(self, lidar_d_front, path_idx, velocity_scale_side):
        add_speed = 0.0
        ref_velocity = 18.0 + add_speed
        velocity_scale_path = self.velocity_scale_path_dict(path_idx, add_speed, ref_velocity)
        min_v = 8.0 * velocity_scale_side
        max_v = 20.0 * velocity_scale_side * velocity_scale_path
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
            print('straight lidar_d_front < min_d')
        elif min_d <= lidar_d_front < max_d:
            error = lidar_d_front - criteria_d
            p_control = p_gain * error
            i_control += i_gain * error * controlTime
            d_control = d_gain * (error - prev_error) / controlTime
            output = p_control + i_control + d_control
            prev_error = error
            result_v = criteria_v + output
            if 0 < result_v <= max_v:
                target_velocity = result_v
            elif result_v <= 0:
                target_velocity = 0
                brake = 100
                print('straight between result_v <= 0')
            else:
                target_velocity = max_v
            if target_velocity < 5.0:
                print('stragiht between target_velocity < 5.0')
                target_velocity = 0.0
                brake = 100
        else:
            target_velocity = max_v
        return (target_velocity, brake)

    def control_speed_curve(self, lidar_d_front, path_idx, velocity_scale_side):
        add_speed = 0.0
        ref_velocity = 16.0 + add_speed
        velocity_scale_path = self.velocity_scale_path_dict(path_idx, add_speed, ref_velocity)
        min_v = 8.0 * velocity_scale_side
        max_v = 18.0 * velocity_scale_side * velocity_scale_path
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
            print('curve lidar_d_front < min_d')
        elif min_d <= lidar_d_front < max_d:
            error = lidar_d_front - criteria_d
            p_control = p_gain * error
            i_control += i_gain * error * controlTime
            d_control = d_gain * (error - prev_error) / controlTime
            output = p_control + i_control + d_control
            prev_error = error
            result_v = criteria_v + output
            if 0 < result_v <= max_v:
                target_velocity = result_v
            elif result_v <= 0:
                target_velocity = 0
                brake = 100
                print('curve between result_v <= 0')
            else:
                target_velocity = max_v
            if target_velocity < 5.0:
                target_velocity = 0.0
                brake = 100
                print('curve betwwen result_v <= 0')
        else:
            target_velocity = max_v
        return (target_velocity, brake)

    def _gps_sample_quality(
        self,
        gps_status,
        gps_covariance,
        last_gps_in,
        position_type,
        ins_status,
        alignment_state,
        time_since_update,
        last_pva_status_in,
    ):
        now = self.node.get_clock().now()
        if now - last_gps_in > Duration(seconds=self.gps_timeout_sec):
            return False, True, 'GPS /fix timeout'
        if now - last_pva_status_in > Duration(seconds=self.pva_status_timeout_sec):
            return False, True, 'PVA status timeout'
        if gps_status < 0:
            return False, True, 'GPS NO_FIX'
        if time_since_update > self.max_time_since_update:
            return False, True, 'PVA position update stale: %ds' % time_since_update

        # 기존 코드는 status==2이면 공분산을 보지 않았고, status!=2이면
        # 경로 번호에 따라 임계값을 바꿨다. PIM222A에서는 위치 솔루션의
        # 종류와 실제 수평 정확도를 항상 함께 확인한다.
        # cov_threshold = 0.005 if path_path_index == 0 else 0.001
        if not isinstance(gps_covariance, (tuple, list)) or len(gps_covariance) < 2:
            return False, True, 'GPS horizontal covariance missing'
        horizontal_covariance = max(float(gps_covariance[0]), float(gps_covariance[1]))
        if not isfinite(horizontal_covariance):
            return False, True, 'GPS horizontal covariance invalid'
        if gps_status != 2:
            return False, False, 'GPS is not RTK fixed (NavSat status=%d)' % gps_status
        if position_type not in self.allowed_position_types:
            return False, False, 'PVA position type %d is not allowed' % position_type
        if ins_status not in self.allowed_ins_statuses:
            return False, False, 'PVA INS status %d is not ready' % ins_status
        if alignment_state not in self.allowed_alignment_states:
            return False, False, 'PVA alignment state %d is not ready' % alignment_state
        if horizontal_covariance > self.max_horizontal_covariance:
            return False, False, 'GPS horizontal covariance %.6f > %.6f' % (
                horizontal_covariance,
                self.max_horizontal_covariance,
            )
        return True, False, 'GPS ready (type=%d, INS=%d, align=%d, hcov=%.6f)' % (
            position_type,
            ins_status,
            alignment_state,
            horizontal_covariance,
        )

    def gps_status_checker(
        self,
        gps_status,
        gps_covariance,
        last_gps_in,
        position_type,
        ins_status,
        alignment_state,
        time_since_update,
        last_pva_status_in,
    ):
        sample_good, immediate_stop, reason = self._gps_sample_quality(
            gps_status,
            gps_covariance,
            last_gps_in,
            position_type,
            ins_status,
            alignment_state,
            time_since_update,
            last_pva_status_in,
        )
        self.gps_reason = reason

        if sample_good:
            self.gps_bad_cycles = 0
            if not self.gps_ready:
                self.gps_good_cycles += 1
                if self.gps_good_cycles >= self.good_cycles_to_resume:
                    self.gps_ready = True
                    self._log_health('gps', '[PIM222A] GPS READY: ' + reason)
                else:
                    self._log_health(
                        'gps_recovery',
                        '[PIM222A] GPS recovery waiting (%d consecutive control cycles required)' % self.good_cycles_to_resume,
                    )
        else:
            self.gps_good_cycles = 0
            if immediate_stop or not self.gps_ready:
                self.gps_ready = False
                self.gps_bad_cycles = 0
                self._log_health('gps', '[PIM222A] GPS STOP: ' + reason, warning=True)
            else:
                self.gps_bad_cycles += 1
                if self.gps_bad_cycles >= self.bad_cycles_to_stop:
                    self.gps_ready = False
                    self._log_health('gps', '[PIM222A] GPS STOP: ' + reason, warning=True)
                else:
                    self._log_health('gps_transient', '[PIM222A] transient GPS degradation: ' + reason, warning=True)

        self.gps_ready_pub.publish(Bool(data=self.gps_ready))
        return self.gps_ready

    def imu_status_checker(self, imu_covariance, last_imu_in, _path_path_index):
        now = self.node.get_clock().now()
        immediate_stop = False
        if now - last_imu_in > Duration(seconds=self.imu_timeout_sec):
            sample_good = False
            immediate_stop = True
            reason = 'IMU topic timeout'
        elif not isinstance(imu_covariance, (tuple, list)) or len(imu_covariance) < 3:
            sample_good = False
            immediate_stop = True
            reason = 'IMU yaw covariance missing'
        else:
            # 조향에 직접 사용하는 yaw 분산만 검사한다. 기존 max(x,y,z)는
            # 차량의 평면 헤딩과 무관한 roll/pitch 때문에 정지할 수 있었다.
            yaw_covariance = float(imu_covariance[2])
            sample_good = isfinite(yaw_covariance) and yaw_covariance <= self.max_yaw_covariance
            immediate_stop = not isfinite(yaw_covariance)
            reason = 'IMU ready (yaw cov=%.6f)' % yaw_covariance if sample_good else (
                'IMU yaw covariance %.6f > %.6f' % (yaw_covariance, self.max_yaw_covariance)
            )

        self.imu_reason = reason
        if sample_good:
            self.imu_bad_cycles = 0
            if not self.imu_ready:
                self.imu_good_cycles += 1
                if self.imu_good_cycles >= self.good_cycles_to_resume:
                    self.imu_ready = True
                    self._log_health('imu', '[PIM222A] IMU READY: ' + reason)
        else:
            self.imu_good_cycles = 0
            if immediate_stop or not self.imu_ready:
                self.imu_ready = False
                self.imu_bad_cycles = 0
                self._log_health('imu', '[PIM222A] IMU STOP: ' + reason, warning=True)
            else:
                self.imu_bad_cycles += 1
                if self.imu_bad_cycles >= self.bad_cycles_to_stop:
                    self.imu_ready = False
                    self._log_health('imu', '[PIM222A] IMU STOP: ' + reason, warning=True)

        self.imu_ready_pub.publish(Bool(data=self.imu_ready))
        return self.imu_ready

    def isGPSerror(
        self,
        gps_status,
        gps_covariance,
        last_gps_in,
        position_type,
        ins_status,
        alignment_state,
        time_since_update,
        last_pva_status_in,
    ):
        gps_ok = self.gps_status_checker(
            gps_status,
            gps_covariance,
            last_gps_in,
            position_type,
            ins_status,
            alignment_state,
            time_since_update,
            last_pva_status_in,
        )
        if not gps_ok:
            self.hold_stop('GPS: ' + self.gps_reason)
            return 'gps_error'
        else:
            return 'no_error'

    def isIMUerror(self, imu_covariance, last_imu_in, path_path_index):
        imu_ok = self.imu_status_checker(imu_covariance, last_imu_in, path_path_index)
        if not imu_ok:
            self.hold_stop('IMU: ' + self.imu_reason)
            return 'imu_error'
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
        self.lfdDistPub = self.node.create_publisher(Float32, _topic(self.node, '/lfd_distance'), CONTROL_QOS)
        self.pathInfo = {1: [numGlobalPathIn, globalPathIn], 2: [numGloablPathOut, globalPathOut]}
        self.K_in, self.K_out = self.calculate_curvature()

    def calculate_curvature(self):
        K_in = self.process_path(self.pathInfo[1][1])
        K_out = self.process_path(self.pathInfo[2][1])
        return (K_in, K_out)

    def calculate_lfd(self, velocity, path_idx, lane, in_index, out_index, is_line_change=False):
        K = self.K_in[in_index] if lane == 1 else self.K_out[out_index]
        if K < 2.5:
            K = 0
        if is_line_change:
            return 6.0
        if lane == 1:
            lfd = 7.7 + velocity * 0.1 - sqrt(K) * 0.65
        else:
            lfd = 7.7 + velocity * 0.1 - sqrt(K) * 0.65
        # return max(lfd, 4)
        return max(lfd, 4.0)

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

    def purePursuit(self, odom, lane, velocity, heading, path_idx, curWaypointIndxIn, curWaypointIndxOut, is_line_change=False):
        curX = odom.pose.pose.position.x
        curY = odom.pose.pose.position.y
        bound = 150
        steering = 0
        lfd = self.calculate_lfd(velocity, path_idx, lane, curWaypointIndxIn, curWaypointIndxOut, is_line_change)
        if not lfd:
            return None
        # self.lfdDistPub.publish(Float32(data=lfd))
        self.lfdDistPub.publish(Float32(data=float(lfd)))
        curWaypoint = curWaypointIndxIn if lane == 1 else curWaypointIndxOut
        if curWaypoint + bound >= self.pathInfo[lane][0]:
            localWaypoints = list(range(curWaypoint, self.pathInfo[lane][0])) + list(range(0, bound - (self.pathInfo[lane][0] - curWaypoint)))
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
        if curr_path_idx == 4:
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
