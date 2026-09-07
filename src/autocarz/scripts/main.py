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

from sensor_data import LidarSensor, GPS, VehicleState
from online import PathPlanner, SpeedDecision, SteeringDecision, LineChanger, MergeChecker
from vehicle_control import VehicleControler
from utilities import Logger, ConfigManager
from std_msgs.msg import Float32, Bool

class AutonomousVehicleController(Node):

    def __init__(self):
        super().__init__('planner_vector')
        self.lane = 1
        self.config_manager = ConfigManager(self)
        self.globalPathIn = self.config_manager.get_path('in')
        self.globalPathOut = self.config_manager.get_path('out')
        numGlobalPathIn = _get_parameter(self, '/mode/numGlobalPathIn')
        numGlobalPathOut = _get_parameter(self, '/mode/numGlobalPathOut')
        self.logger = Logger(self)
        self.gps_sensor = GPS(self)
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
            odom_back, heading, gps_status_back, gps_covariance_back, checkGPS_back, checkHeading, last_back_gps_in = self.gps_sensor.gps_data_back()
            odom_front, gps_status_front, gps_covariance_front, checkGPS_front, last_front_gps_in = self.gps_sensor.gps_data_front()
            velocity, _, checkERP = self.vehicle_state.vehicle_data()
            checkERP = True
            if (checkGPS_front or checkGPS_back) and checkHeading and checkERP:
                lidar_front, lidar_side_front, lidar_side_back = self.lidar_sensor.lidar_data(self.lane)
                curWaypointIndxIn, curWaypointIndxOut, distanceIn, distanceOut = self.path_planner.localization(odom_back, self.lane)
                in_path_idx = self.globalPathIn.poses[curWaypointIndxIn].pose.orientation.w
                out_path_idx = self.globalPathOut.poses[curWaypointIndxOut].pose.orientation.w
                curr_path_idx, distance = (in_path_idx, distanceIn) if self.lane == 1 else (out_path_idx, distanceOut)
                self.path_idx_pub.publish(Float32(data=curr_path_idx))
                gps_error_status = self.speed_decision.isGPSerror(gps_status_back, gps_covariance_back, last_back_gps_in, gps_status_front, gps_covariance_front, last_front_gps_in, curr_path_idx)
                if gps_error_status == 'both_error':
                    self.get_logger().warning('Both GPS Error. Stop.')
                    continue
                elif gps_error_status == 'back_error':
                    self.get_logger().warning('Back GPS Error. Using Front GPS.')
                    continue
                elif gps_error_status == 'front_error':
                    self.get_logger().warning('Front GPS Error. Using Back GPS.')
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
                    target_velocity = min(target_velocity, 10)
                steering_angle = self.steering_decision.purePursuit(odom_back, self.lane, velocity, heading, curr_path_idx, curWaypointIndxIn, curWaypointIndxOut, is_line_changing)
                print(steering_angle)
                self.vehicle_control.control(target_velocity, brake, steering_angle)
                self.config_manager.broadcast(odom_back)
                self.config_manager.pub_path()
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
