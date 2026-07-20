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
from online_PG import PathPlanner, SpeedDecision, SteeringDecision, LineChanger, MergeChecker
from vehicle_control import VehicleControler
from utilities import Logger, ConfigManager
from GeneratePath import PathGenerater, PathEvaluator, Path_util
from std_msgs.msg import Int32
import time
IN_WIDTH = 1.7
OUT_WIDTH = -1.7

class AutonomousVehicleController(Node):

    def __init__(self):
        super().__init__('planner_vector')
        numGlobalPathIn = 4544
        numGlobalPathOut = 4549
        self.config_manager = ConfigManager(self)
        self.globalPathIn = self.config_manager.get_path('in')
        self.globalPathOut = self.config_manager.get_path('out')
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
        self.rate = self.create_rate(10)
        self.curWaypointIdxIn_pub = self.create_publisher(Int32, _topic(self, 'curWaypointIdxIn'), CONTROL_QOS)
        self.curWaypointIdxOut_pub = self.create_publisher(Int32, _topic(self, 'curWaypointIdxOut'), CONTROL_QOS)
        self.in_path = Path_util()
        self.out_path = Path_util()
        in_path_data = self.in_path.path_read('middle_in_240827.txt')
        out_path_data = self.out_path.path_read('middle_out_240827.txt')
        self.PathGenerator_in = PathGenerater(self, in_path_data)
        self.PathEvaluator_in = PathEvaluator(self, in_path_data)
        self.PathGenerator_out = PathGenerater(self, out_path_data)
        self.PathEvaluator_out = PathEvaluator(self, out_path_data)

    def run(self):
        target_velocity = 0
        is_line_changing = False
        path_in = False
        pre_d = None
        curWaypointIdxIn = 0
        curWaypointIdxOut = 0
        while rclpy.ok():
            odom_back, heading, gps_status_back, gps_covariance_back, checkGPS_back, checkHeading, last_back_gps_in = self.gps_sensor.gps_data_back()
            odom_front, gps_status_front, gps_covariance_front, checkGPS_front, last_front_gps_in = self.gps_sensor.gps_data_front()
            velocity, _, checkERP = self.vehicle_state.vehicle_data()
            if (checkGPS_front or checkGPS_back) and checkHeading and checkERP:
                curWaypointIndxIn, curWaypointIndxOut, distanceIn, distanceOut = self.path_planner.localization(odom_back)
                in_path_idx = self.globalPathIn.poses[curWaypointIndxIn].pose.orientation.w
                out_path_idx = self.globalPathOut.poses[curWaypointIndxOut].pose.orientation.w
                gps_error_status = self.speed_decision.isGPSerror(gps_status_back, gps_covariance_back, last_back_gps_in, gps_status_front, gps_covariance_front, last_front_gps_in, 0)
                if gps_error_status == 'both_error':
                    self.get_logger().warning('Both GPS Error. Stop.')
                    continue
                elif gps_error_status == 'back_error':
                    self.get_logger().warning('Back GPS Error. Using Front GPS.')
                    continue
                elif gps_error_status == 'front_error':
                    self.get_logger().warning('Front GPS Error. Using Back GPS.')
                if path_in:
                    curr_path_idx = in_path_idx
                    self.curWaypointIdxIn_pub.publish(Int32(data=int(curWaypointIndxIn)))
                else:
                    curr_path_idx = out_path_idx
                    self.curWaypointIdxOut_pub.publish(Int32(data=int(curWaypointIndxOut)))
                if curr_path_idx == 4 and pre_d == IN_WIDTH:
                    path_in = True
                elif curr_path_idx == 4 and pre_d == OUT_WIDTH:
                    path_in = False
                if path_in:
                    if curr_path_idx == 2:
                        generated_paths, velocity_values, d_values, diff_d = self.PathGenerator_in.generate_path([6, 8, 10, 12], [0.0], [odom_back.pose.pose.position.x, odom_back.pose.pose.position.y], heading)
                    elif curr_path_idx == 3:
                        generated_paths, velocity_values, d_values, diff_d = self.PathGenerator_in.generate_path([6, 8, 10, 12], [IN_WIDTH], [odom_back.pose.pose.position.x, odom_back.pose.pose.position.y], heading)
                    elif curr_path_idx == 4:
                        generated_paths, velocity_values, d_values, diff_d = self.PathGenerator_in.generate_path([6, 8], [IN_WIDTH], [odom_back.pose.pose.position.x, odom_back.pose.pose.position.y], heading)
                    else:
                        generated_paths, velocity_values, d_values, diff_d = self.PathGenerator_in.generate_path([6, 8, 10, 12], [IN_WIDTH, OUT_WIDTH], [odom_back.pose.pose.position.x, odom_back.pose.pose.position.y], heading)
                elif curr_path_idx == 2:
                    generated_paths, velocity_values, d_values, diff_d = self.PathGenerator_out.generate_path([12], [0.0], [odom_back.pose.pose.position.x, odom_back.pose.pose.position.y], heading)
                elif curr_path_idx == 3:
                    generated_paths, velocity_values, d_values, diff_d = self.PathGenerator_out.generate_path([12], [OUT_WIDTH], [odom_back.pose.pose.position.x, odom_back.pose.pose.position.y], heading)
                elif curr_path_idx == 4:
                    generated_paths, velocity_values, d_values, diff_d = self.PathGenerator_out.generate_path([8], [OUT_WIDTH], [odom_back.pose.pose.position.x, odom_back.pose.pose.position.y], heading)
                else:
                    generated_paths, velocity_values, d_values, diff_d = self.PathGenerator_out.generate_path([12], [IN_WIDTH, OUT_WIDTH], [odom_back.pose.pose.position.x, odom_back.pose.pose.position.y], heading)
                optimal_path, optimal_velocity, optimal_d, e_err = self.PathEvaluator_in.calc_cost(generated_paths, velocity_values, d_values, [odom_back.pose.pose.position.x, odom_back.pose.pose.position.y], curr_path_idx, diff_d)
                pre_d = optimal_d if optimal_d else None
                if optimal_velocity == None:
                    print('optimal_path is None')
                    optimal_velocity = 0.0
                    brake = 100
                else:
                    brake = 0
                steering_angle = self.steering_decision.purePursuit(odom_back, velocity, heading, e_err, optimal_path, curr_path_idx)
                self.vehicle_control.control(optimal_velocity * 3.6, brake, steering_angle)
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
