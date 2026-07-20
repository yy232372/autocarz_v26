#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from ament_index_python.packages import get_package_share_directory

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

import os
import numpy as np
from math import pi, cos, sin, sqrt
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Float32MultiArray
from scipy.spatial import KDTree
BETWEENPOINT = 0.1
YAW_INDEX = 2
FRONT = 0.1
TIME_PLANNING = 3
TIME_PLANNING_OBSTACLE = 3
DEGREE = 10
ROBOT_RADIUS = 3.5

def euclidean_distance(v1, v2):
    return sqrt(sum([(a - b) ** 2 for a, b in zip(v1, v2)]))

class BasePath(object):

    def __init__(self, global_path):
        self.global_path = self.extract_coordinates_from_path(global_path)
        self.global_path = self.add_angles_to_path(self.global_path, len(self.global_path))
        self.MAX_CURVATURE = 0.5

    def extract_coordinates_from_path(self, path_msg):
        poses = path_msg.poses
        num_poses = len(poses)
        x_coords = []
        y_coords = []
        for pose in poses:
            x_coords.append(pose.pose.position.x)
            y_coords.append(pose.pose.position.y)
        x_array = np.array(x_coords)
        y_array = np.array(y_coords)
        return np.vstack((x_array, y_array)).T

    def add_angles_to_path(self, global_path, num_points):
        angles = np.zeros((num_points, 1))
        for i in range(num_points - 1):
            dx = global_path[i + 1, 0] - global_path[i, 0]
            dy = global_path[i + 1, 1] - global_path[i, 1]
            angle = np.arctan2(dy, dx)
            angles[i, 0] = angle
        dx_last = global_path[0, 0] - global_path[-1, 0]
        dy_last = global_path[0, 1] - global_path[-1, 1]
        last_angle = np.arctan2(dy_last, dx_last)
        angles[-1, 0] = last_angle
        path_with_angles = np.hstack((global_path, angles))
        return path_with_angles

    def normalize_angle(self, angle, lower_bound=-np.pi / 9, upper_bound=np.pi / 9):
        """
        Normalize an angle to be within the range [-pi, pi).
        """
        angle = (angle + np.pi) % (2 * np.pi) - np.pi
        angle = np.clip(angle, lower_bound, upper_bound)
        return angle

    def quintic_polynomial(self, xs, ys, vs, as_, xf, yf, vf, af):
        A = np.array([[1, xs, xs ** 2, xs ** 3, xs ** 4, xs ** 5], [0, 1, 2 * xs, 3 * xs ** 2, 4 * xs ** 3, 5 * xs ** 4], [0, 0, 2, 6 * xs, 12 * xs ** 2, 20 * xs ** 3], [1, xf, xf ** 2, xf ** 3, xf ** 4, xf ** 5], [0, 1, 2 * xf, 3 * xf ** 2, 4 * xf ** 3, 5 * xf ** 4], [0, 0, 2, 6 * xf, 12 * xf ** 2, 20 * xf ** 3]])
        B = np.array([ys, vs, as_, yf, vf, af])
        coeffs = np.linalg.solve(A, B)
        return coeffs

class PathGenerater(BasePath):

    def __init__(self, node: Node, path):
        self.node = node
        self._subscriptions = []
        super(PathGenerater, self).__init__(path)
        self.lattice_path = Path()
        self.lattice_path.header.frame_id = 'map'
        self.read_pose = PoseStamped()
        self.read_pose.pose.position.x = 0
        self.read_pose.pose.position.y = 0
        self.read_pose.pose.position.z = 0
        self.read_pose.pose.orientation.x = 0
        self.read_pose.pose.orientation.y = 0
        self.read_pose.pose.orientation.z = 0
        self.read_pose.pose.orientation.w = 1
        self.generated_paths = []
        self.publishers = {}

    def generate_path(self, s, d, ego, heading):
        lines = []
        s_values = []
        d_values = []
        diff_x = []
        diff = ego - self.global_path[:, :2]
        distances = np.linalg.norm(diff, axis=1)
        closest_index = np.argmin(distances)
        diff_p = diff[closest_index]
        yaw_p = self.global_path[closest_index, YAW_INDEX] + pi / 2
        ego_e = cos(yaw_p) * diff_p[0] + sin(yaw_p) * diff_p[1]
        self.clear_old_paths(len(s) * len(d))
        for dd in d:
            for ss in s:
                generated_path, generated_s, generated_d, x = self.one_path_generate(ego_e, dd, ss / 3.6 * TIME_PLANNING, closest_index, heading)
                if generated_path is not None and generated_s is not None:
                    lines.append(generated_path)
                    s_values.append(generated_s)
                    d_values.append(generated_d)
                    diff_x.append(x)
        for i, path in enumerate(lines):
            globals()['lattice_pub_{}'.format(i + 1)] = self.node.create_publisher(Path, '/generated_path/lattice_path_{}'.format(i + 1), CONTROL_QOS)
            globals()['lattice_pub_{}'.format(i + 1)].publish(path)
        return (lines, s_values, d_values, diff_x)

    def one_path_generate(self, ego_e, d, s, closest_index, heading):
        self.lattice_path = Path()
        self.lattice_path.header.frame_id = 'map'
        slope = self.normalize_angle(heading - self.global_path[closest_index, YAW_INDEX])
        x_n = np.arange(FRONT, s, BETWEENPOINT)
        e = d - ego_e
        e = np.clip(e, -3.4, 3.4)
        e = abs(e)
        coeffs = self.quintic_polynomial(FRONT, ego_e, 0, 0, s, d, 0, 0)
        y_n = np.polyval(coeffs[::-1], x_n)
        point_n = len(x_n)
        x_new = np.zeros(point_n)
        y_new = np.zeros(point_n)
        start = int(FRONT / BETWEENPOINT)
        for i in range(start, point_n + start):
            read_pose = PoseStamped()
            read_pose.header.frame_id = 'map'
            read_pose.pose.position.z = 0
            read_pose.pose.orientation.z = 0
            read_pose.pose.orientation.w = 1
            trans_x = self.global_path[(closest_index + i) % len(self.global_path), 0]
            trans_y = self.global_path[(closest_index + i) % len(self.global_path), 1]
            trans_theta = self.global_path[(closest_index + i) % len(self.global_path), YAW_INDEX]
            x = trans_x + -y_n[i - start] * np.sin(trans_theta)
            y = trans_y + y_n[i - start] * np.cos(trans_theta)
            x_new[i - start] = x
            y_new[i - start] = y
            read_pose.pose.position.x = x
            read_pose.pose.position.y = y
            self.lattice_path.poses.append(read_pose)
        return (self.lattice_path, s, d, e)

    def clear_old_paths(self, num_paths):
        empty_path = Path()
        empty_path.header.frame_id = 'map'
        for i in range(num_paths):
            topic_name = '/generated_path/lattice_path_{}'.format(i + 1)
            if topic_name not in self.publishers:
                self.publishers[topic_name] = self.node.create_publisher(Path, topic_name, CONTROL_QOS)
            self.publishers[topic_name].publish(empty_path)

class PathEvaluator(BasePath):

    def __init__(self, node: Node, path):
        self.node = node
        self._subscriptions = []
        super(PathEvaluator, self).__init__(path)
        self.optimal_path_pub = self.node.create_publisher(Path, _topic(self.node, '/optimal_local_path1'), CONTROL_QOS)
        self.obstacles = np.array([])
        self._subscriptions.append(self.node.create_subscription(Float32MultiArray, _topic(self.node, '/obstacle_tracking'), self.obstacle_poses_callback, CONTROL_QOS))
        self.obstacle_position = np.array([])
        self.previous_d = None
        self.lane_change_flag = False

    def obstacle_poses_callback(self, data):
        if data is None or len(data.data) == 0:
            self.obstacle_position = np.array([])
            return
        temp = np.array(data.data)
        if temp.size % (2 * (1 + TIME_PLANNING_OBSTACLE)) != 0:
            self.obstacle_position = np.array([])
            return
        self.obstacle_position = temp.reshape(-1, 2 * (1 + TIME_PLANNING_OBSTACLE))

    def calc_cost(self, generated_paths, s_values, d_values, ego_position, curr_path_idx, diff_d):
        """
        입력: 생성된 전체 lattice path, 전역 좌표 장애물
        출력: 최적의 lattice path 하나
        과정:
            - 곡률 계산 -> 곡률 제한 필터링
            - 최적 경로 찾기 -> cost value 기반 (경로 길이, 경로 부드러움, 전역 경로와의 유사성, (장애물과의 거리))
        """
        fine_path = []
        if not generated_paths:
            self.node.get_logger().warning('No valid paths found')
            return (None, None, None, None, None, None)
        if curr_path_idx == 0:
            for path, s, d, e in zip(generated_paths, s_values, d_values, diff_d):
                check = self.obstacle_filtering(path, ego_position)
                if check:
                    fine_path.append((path, s, d, e))
        elif curr_path_idx == 2:
            for path, s, d, e in zip(generated_paths, s_values, d_values, diff_d):
                check = self.obstacle_filtering_rotary(path, ego_position)
                if check:
                    fine_path.append((path, s, d, e))
        else:
            for path, s, d, e in zip(generated_paths, s_values, d_values, diff_d):
                check = self.obstacle_filtering_curve(path, ego_position)
                if check:
                    fine_path.append((path, s, d, e))
        if not len(fine_path):
            return (None, None, None, None)
        optimal_velocity = 0
        optimal_d = 0
        optimal_path = None
        optimal_err = 0
        if curr_path_idx == 0:
            for x in fine_path:
                if optimal_velocity < x[1]:
                    optimal_path = x[0]
                    optimal_velocity = x[1]
                    optimal_d = x[2]
                    optimal_err = x[3]
                elif x[1] == optimal_velocity and x[2] == self.previous_d:
                    optimal_path = x[0]
                    optimal_velocity = x[1]
                    optimal_d = x[2]
                    optimal_err = x[3]
            self.previous_d = optimal_d
        elif curr_path_idx == 1:
            for x in fine_path:
                if self.previous_d is None:
                    self.previous_d = x[2]
                    optimal_d = x[2]
                    optimal_err = x[3]
                if x[2] == self.previous_d:
                    if optimal_velocity < x[1]:
                        optimal_path = x[0]
                        optimal_velocity = x[1]
                        optimal_err = x[3]
        elif curr_path_idx == 3 or curr_path_idx == 4:
            for x in fine_path:
                if optimal_velocity < x[1]:
                    optimal_path = x[0]
                    optimal_velocity = x[1]
                    optimal_d = x[2]
                    optimal_err = x[3]
                    self.previous_d = optimal_d
        elif curr_path_idx == 2:
            for x in fine_path:
                if optimal_velocity < x[1]:
                    optimal_path = x[0]
                    optimal_velocity = x[1]
                    optimal_err = x[3]
        if optimal_path is None:
            return (None, None, None, None)
        self.publish_optimal_path(optimal_path)
        return (optimal_path, optimal_velocity / TIME_PLANNING, optimal_d, optimal_err)

    def obstacle_filtering(self, path, ego_position):
        obstacle_position = self.obstacle_position
        if len(obstacle_position) == 0:
            return True
        path_length = len(path.poses)
        ego_positions = [ego_position]
        for t in range(1, TIME_PLANNING + 1):
            index = int(t * path_length / TIME_PLANNING)
            ego_positions.append([path.poses[min(index, path_length - 1)].pose.position.x, path.poses[min(index, path_length - 1)].pose.position.y])
        collision_threshold = ROBOT_RADIUS
        obs = []
        for t in range(TIME_PLANNING_OBSTACLE + 1):
            obs.append(obstacle_position[:, t * 2:t * 2 + 2])
        for i, ego_future_position in enumerate(ego_positions[1:4]):
            for x in range(0, i + 2):
                temp = obs[x]
                for j in range(temp.shape[0]):
                    if np.linalg.norm(np.array(ego_future_position) - temp[j]) < collision_threshold:
                        return False
        return True

    def obstacle_filtering_curve(self, path, ego_position):
        obstacle_position = self.obstacle_position
        if len(obstacle_position) == 0:
            return True
        path_length = len(path.poses)
        ego_positions = [ego_position]
        for t in range(1, TIME_PLANNING + 1):
            index = int(t * path_length / TIME_PLANNING)
            ego_positions.append([path.poses[min(index, path_length - 1)].pose.position.x, path.poses[min(index, path_length - 1)].pose.position.y])
        collision_threshold = ROBOT_RADIUS
        obs_position_0 = obstacle_position[:, 0:2]
        for i, ego_future_position in enumerate(ego_positions[1:4]):
            temp = obs_position_0
            for j in range(obs_position_0.shape[0]):
                if np.linalg.norm(np.array(ego_future_position) - temp[j]) < collision_threshold:
                    return False
        return True

    def obstacle_filtering_rotary(self, path, ego_position):
        obstacle_position = self.obstacle_position
        if len(obstacle_position) == 0:
            return True
        path_length = len(path.poses)
        ego_positions = [ego_position]
        for t in range(1, TIME_PLANNING + 1):
            index = int(t * path_length / TIME_PLANNING)
            ego_positions.append([path.poses[min(index, path_length - 1)].pose.position.x, path.poses[min(index, path_length - 1)].pose.position.y])
        collision_threshold = ROBOT_RADIUS + 1.0
        obs_position_0 = obstacle_position[:, 0:2]
        for i, ego_future_position in enumerate(ego_positions[1:4]):
            temp = obs_position_0
            for j in range(obs_position_0.shape[0]):
                if np.linalg.norm(np.array(ego_future_position) - temp[j]) < collision_threshold:
                    return False
        return True

    def publish_optimal_path(self, best_path):
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.node.get_clock().now().to_msg()
        path_msg.poses = best_path.poses
        self.optimal_path_pub.publish(path_msg)

class Path_util:

    def __init__(self):
        self.x_list = []
        self.y_list = []
        self.index = []

    def path_read(self, path_file):
        path_dir = os.path.join(get_package_share_directory('autocarz'), 'path')
        path = Path()
        path.header.frame_id = 'map'
        with open(os.path.join(path_dir, path_file), 'r') as file:
            lines = file.readlines()
            for line in lines:
                data = line.split()
                pose = PoseStamped()
                pose.pose.position.x = float(data[0])
                self.x_list.append(pose.pose.position.x)
                pose.pose.position.y = float(data[1])
                self.y_list.append(pose.pose.position.y)
                pose.pose.position.z = 0.0
                pose.pose.orientation.w = float(data[2])
                self.index.append(pose.pose.orientation.w)
                path.poses.append(pose)
        return path

    def find_nearest_waypoint(self, cur_x, cur_y):
        points = np.array(list(zip(self.x_list, self.y_list)))
        tree = KDTree(points)
        _, nearest_idx = tree.query([cur_x, cur_y])
        return (int(nearest_idx), int(self.index[nearest_idx]))
