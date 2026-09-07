#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import re as _re
import threading
from typing import Any as _Any
import rclpy
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
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

def _topic(node: Node, default_topic: str) -> str:
    slug = default_topic.strip('/')
    slug = _re.sub(r'[^A-Za-z0-9_]', '_', slug)
    slug = _re.sub(r'_+', '_', slug).strip('_') or 'root'
    parameter_name = f'topics.{slug}'
    if not node.has_parameter(parameter_name):
        node.declare_parameter(parameter_name, default_topic)
    return str(node.get_parameter(parameter_name).value)

from sensor_data_isro import LidarSensor, ISRO, VehicleState
from online_isro import PathPlanner, SpeedDecision, SteeringDecision, LineChanger, MergeChecker
from vehicle_control import VehicleControler
from utilities import Logger, ConfigManager
from std_msgs.msg import Float32, Bool

class AutonomousVehicleController(Node):

    def __init__(self):
        super().__init__(
            'planner_vector',
            automatically_declare_parameters_from_overrides=True,
        )
        self.lane = 1
        self.config_manager = ConfigManager(self)
        self.globalPathIn = self.config_manager.get_path('in')
        self.globalPathOut = self.config_manager.get_path('out')
        # 경로는 GPS/IMU/ERP42 상태와 무관한 정적 데이터다. 시작 시 한 번
        # 발행하고 transient-local QoS로 RViz에 유지한다.
        self.config_manager.pub_path()
        numGlobalPathIn = len(self.globalPathIn.poses)
        numGlobalPathOut = len(self.globalPathOut.poses)
        self.logger = Logger(self)
        self.gps_sensor = ISRO(self)
        self.lidar_sensor = LidarSensor(self)
        self.vehicle_state = VehicleState(self)
        self.path_planner = PathPlanner(self, self.globalPathIn, self.globalPathOut)
        self.speed_decision = SpeedDecision(self)
        self.steering_decision = SteeringDecision(self, numGlobalPathIn, numGlobalPathOut, self.globalPathIn, self.globalPathOut)
        self.line_changer = LineChanger(self)
        self.merge_checker = MergeChecker()
        self.vehicle_control = VehicleControler(self)
        self.path_idx_pub = self.create_publisher(Float32, _topic(self, '/curr_path_idx'), CONTROL_QOS)
        self.line_changing_pub = self.create_publisher(Bool, _topic(self, '/is_line_changing'), CONTROL_QOS)
        self.rate = self.create_rate(10)

    def run(self):
        target_velocity = 0
        is_line_changing = False
        while rclpy.ok():
            self.gps_sensor.make_odom()
            gps_status, gps_covariance, checkGPS, last_gps_in = self.gps_sensor.gps_data()
            odom, heading, checkHeading, imu_covariance, checkIMU, last_imu_in = self.gps_sensor.imu_data()
            position_type, ins_status, alignment_state, time_since_update, checkPVAStatus, last_pva_status_in = self.gps_sensor.pva_status_data()
            velocity, _, checkERP = self.vehicle_state.vehicle_data()

            # RViz/센서 처리를 위한 TF는 ERP42 연결 및 주행 허가와 분리한다.
            # 위치와 자세 메시지가 최근에 들어온 경우에만 방송하므로, 제어가
            # WAITING/STOP 상태여도 map -> base_link TF가 끊기지 않는다.
            now = self.get_clock().now()
            navigation_fresh = (
                checkGPS
                and checkIMU
                and checkHeading
                and now - last_gps_in <= Duration(seconds=0.3)
                and now - last_imu_in <= Duration(seconds=0.2)
            )
            if navigation_fresh:
                self.config_manager.broadcast(odom)

            sensors_initialized = checkGPS and checkIMU and checkHeading and checkPVAStatus and checkERP
            if not sensors_initialized:
                waiting = []
                if not checkGPS:
                    waiting.append('/fix')
                if not checkIMU or not checkHeading:
                    waiting.append('/imu/data')
                if not checkPVAStatus:
                    waiting.append('/pva/* status')
                if not checkERP:
                    waiting.append('/erp42/state')
                self.speed_decision.hold_stop('WAITING: ' + ', '.join(waiting))
                self.rate.sleep()
                continue

            # 센서 품질을 먼저 통과시킨 뒤에만 위치 기반 경로/라이다 판단을
            # 수행한다. PIM222A 상태 판정은 경로 번호에 의존하지 않는다.
            gps_error_status = self.speed_decision.isGPSerror(
                gps_status,
                gps_covariance,
                last_gps_in,
                position_type,
                ins_status,
                alignment_state,
                time_since_update,
                last_pva_status_in,
            )
            if gps_error_status == 'gps_error':
                # self.get_logger().warning('GPS Error. Stop.')
                # 기존 continue는 아래 10 Hz rate.sleep()을 건너뛰어 같은
                # 오류를 초당 수백 번 출력하고 제동 명령을 폭주시켰다.
                self.rate.sleep()
                continue

            imu_error_status = self.speed_decision.isIMUerror(imu_covariance, last_imu_in, 0)
            if imu_error_status == 'imu_error':
                # self.get_logger().warning('IMU Error. Stop.')
                self.rate.sleep()
                continue

            if sensors_initialized:
                lidar_front, lidar_side_front, lidar_side_back = self.lidar_sensor.lidar_data(self.lane)
                curWaypointIndxIn, curWaypointIndxOut, distanceIn, distanceOut = self.path_planner.localization(odom, self.lane)
                in_path_idx = self.globalPathIn.poses[curWaypointIndxIn].pose.orientation.w
                out_path_idx = self.globalPathOut.poses[curWaypointIndxOut].pose.orientation.w
                curr_path_idx, distance = (in_path_idx, distanceIn) if self.lane == 1 else (out_path_idx, distanceOut)
                self.path_idx_pub.publish(Float32(data=float(curr_path_idx)))
                velocity_scale_side = self.merge_checker.merge_check(curr_path_idx, lidar_side_front, lidar_side_back)
                change, _, is_line_changing = self.line_changer.line_change_check(curr_path_idx, distance, target_velocity, lidar_front, lidar_side_front, lidar_side_back, self.lane)
                if change:
                    self.lane = 1 if self.lane == 2 else 2
                self.line_changing_pub.publish(Bool(data=is_line_changing))
                if curr_path_idx == 1 or curr_path_idx == 2:
                    target_velocity, brake = self.speed_decision.control_speed_curve(lidar_front, curr_path_idx, velocity_scale_side)
                else:
                    target_velocity, brake = self.speed_decision.control_speed(lidar_front, curr_path_idx, velocity_scale_side)
                if is_line_changing:
                    target_velocity = min(target_velocity, 20)
                steering_angle = self.steering_decision.purePursuit(odom, self.lane, velocity, heading, curr_path_idx, curWaypointIndxIn, curWaypointIndxOut, is_line_changing)
                print(steering_angle)
                self.speed_decision.mark_control_ready()
                self.vehicle_control.control(target_velocity, brake, steering_angle)
                # 기존에는 주행 제어가 허가된 경우에만 TF를 방송했다.
                # self.config_manager.broadcast(odom)
                # 기존에는 정상 주행 중에만 전체 경로를 10 Hz로 재발행했다.
                # self.config_manager.pub_path()
            self.rate.sleep()

def main(args=None):
    rclpy.init(args=args)
    node = AutonomousVehicleController()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=1.0)

if __name__ == '__main__':
    main()
