#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import re
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from std_msgs.msg import Float32MultiArray, Header
from geometry_msgs.msg import PoseArray, Pose
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from filterpy.kalman import KalmanFilter
from scipy.spatial.distance import cdist

CONTROL_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)
DT = 0.5

def _topic(node: Node, default_topic: str) -> str:
    slug = re.sub(r'[^A-Za-z0-9_]', '_', default_topic.strip('/'))
    slug = re.sub(r'_+', '_', slug).strip('_') or 'root'
    parameter_name = f'topics.{slug}'
    if not node.has_parameter(parameter_name):
        node.declare_parameter(parameter_name, default_topic)
    return str(node.get_parameter(parameter_name).value)

class ObjectTracker(Node):
    def __init__(self):
        super().__init__('tracker')
        self.positions = None
        self.obtracker = Tracker(self, 5, 2, 2.3)
        self.detect_sub = self.create_subscription(
            Float32MultiArray,
            _topic(self, '/cluster/obstacle'),
            self.detect_data_callback,
            CONTROL_QOS,
        )
        self.tracking_pub = self.create_publisher(
            Float32MultiArray, _topic(self, '/obstacle_tracking'), CONTROL_QOS
        )
        self.tracking_pose_pub = self.create_publisher(
            PoseArray, _topic(self, '/obstacle/poses'), CONTROL_QOS
        )
        self.tracking_point_pub = self.create_publisher(
            PointCloud2, _topic(self, '/obstacle/point'), qos_profile_sensor_data
        )
        self.timer = self.create_timer(0.1, self.process_tracking)

    def process_tracking(self):
        if self.positions is None:
            return
        trackers = self.obtracker.update(self.positions)
        pose_array = PoseArray()
        pose_array.header.stamp = self.get_clock().now().to_msg()
        pose_array.header.frame_id = 'map'
        points = []
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'map'
        obstacle_tracking = Float32MultiArray()
        obstacle_list = []

        for data in trackers:
            pose = Pose()
            pose.position.x = float(data[0])
            pose.position.y = float(data[1])
            pose.position.z = 0.0
            yaw = np.arctan2(data[3], data[2])
            quaternion = quaternion_from_euler(0.0, 0.0, yaw)
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = quaternion
            pose_array.poses.append(pose)
            obstacle_list.extend([float(data[0]), float(data[1])])

            for step in (1, 2, 3):
                predicted_x = float(data[0] + data[2] * DT * step)
                predicted_y = float(data[1] + data[3] * DT * step)
                points.append([predicted_x, predicted_y, 0.0])
                obstacle_list.extend([predicted_x, predicted_y])

        cloud_msg = point_cloud2.create_cloud_xyz32(header, points)
        obstacle_tracking.data = obstacle_list
        self.tracking_pub.publish(obstacle_tracking)
        self.tracking_pose_pub.publish(pose_array)
        self.tracking_point_pub.publish(cloud_msg)

    def detect_data_callback(self, data: Float32MultiArray):
        temp_pos = np.asarray(data.data, dtype=float)
        if temp_pos.size == 0:
            self.positions = np.empty((0, 2))
            return
        if temp_pos.size % 6 != 0:
            self.get_logger().warning(
                f'장애물 배열 길이 {temp_pos.size}는 6의 배수가 아니므로 해당 프레임을 무시합니다.'
            )
            return
        self.positions = temp_pos.reshape((-1, 6))[:, :2]

class Tracker:
    def __init__(self, node: Node, max_age=1, min_hits=3, dist_threshold=0.2):
        self.node = node
        self.max_age = max_age                  # 추적이 중단된 후 트랙을 유지할 최대 프레임 수
        self.min_hits = min_hits                # 새로운 객체가 트랙으로 인식되기 전에 필요한 최소 감지 횟수
        self.dist_threshold = dist_threshold    # 거리 매칭 임계값
        self.trackers = []                      # 현재 추적 중인 모든 객체를 관리하는 리스트
        self.frame_count = 0                    # 현재까지 처리된 프레임 수

    def update(self, dets = np.empty((0, 2))):
        self.frame_count += 1
        trks = np.zeros((len(self.trackers), 2))
        to_del = []
        ret = []

        for t, trk in enumerate(trks):
            pos = self.trackers[t].predict()
            trk[:] = [pos[0][0], pos[1][0]] if pos[0].shape else [pos[0], pos[1]]
            if np.any(np.isnan(pos)):
                to_del.append(t)
        trks = np.ma.compress_rows(np.ma.masked_invalid(trks))

        for t in reversed(to_del):
            self.trackers.pop(t)

        matched, unmatched_dets, unmatched_trks = associate_detections_to_trackers(dets, trks, self.dist_threshold)

        for m in matched:
            self.trackers[m[1]].update(dets[m[0],:])

        for i in unmatched_dets:
            trk = KalmanPositionTracker(self.node, dets[i, :])
            self.trackers.append(trk)
        i = len(self.trackers)
        for trk in reversed(self.trackers):
            d = trk.get_state()
            if (trk.time_since_update < 1) and (trk.hit_streak >= self.min_hits or self.frame_count <= self.min_hits):
                ret.append(np.concatenate((d, np.array([[trk.id + 1]]))).reshape(1, -1))
            i -= 1
            if (trk.time_since_update > self.max_age):
                self.trackers.pop(i)
        if (len(ret) > 0):
            return np.concatenate(ret)
        return np.empty((0,5))

def associate_detections_to_trackers(detections, trackers, dist_threshold = 0.2):
    """
    감지된 객체와 추적 중인 객체를 매칭하여, 각 객체에 대해 추적을 유지하거나
    새로운 추적기를 생성시키는 함수
    """
    if len(trackers) == 0:
        return np.empty((0, 2), dtype = int), np.arange(len(detections)), np.empty((0, 2), dtype=int)

    distance_matrix = cdist(detections, trackers, metric='euclidean')

    if min(distance_matrix.shape) > 0:
        a = (distance_matrix < dist_threshold).astype(np.int32)
        if a.sum(1).max() == 1 and a.sum(0).max() == 1: # 1:1 매칭이 된 경우
            matched_indices = np.stack(np.where(a), axis=1)
        else:
            matched_indices = linear_assignment(distance_matrix)
    else:
        matched_indices = np.empty(shape = (0, 2))

    unmatched_detections = []
    for d, det in enumerate(detections):
        if (d not in matched_indices[:, 0]):
            unmatched_detections.append(d)
    
    unmatched_trackers = []
    for t, trk in enumerate(trackers):
        if (t not in matched_indices[:, 1]):
            unmatched_trackers.append(t)

    # dist가 큰 매칭을 필터링
    matches = []
    for m in matched_indices:
        if (distance_matrix[m[0], m[1]] > dist_threshold):
            unmatched_detections.append(m[0])
            unmatched_trackers.append(m[1])
        else:
            matches.append(m.reshape(1, 2))
    if len(matches) == 0:
        matches = np.empty((0, 2), dtype = int)
    else:
        matches = np.concatenate(matches, axis = 0)
    
    return matches, np.array(unmatched_detections), np.array(unmatched_trackers)

def linear_assignment(cost_matrix):
    try:
        import lap
        _, x, y = lap.lapjv(cost_matrix, extend_cost=True)
        return np.array([[y[i], i] for i in x if i >= 0])
    except ImportError:
        from scipy.optimize import linear_sum_assignment
        x, y = linear_sum_assignment(cost_matrix)
        return np.array(list(zip(x, y)))

class KalmanPositionTracker:
    count = 0
    def __init__(self, node: Node, position):
        self.node = node
        self.kf = KalmanFilter(dim_x = 4, dim_z = 2)
        self.kf.F = np.eye(4)
        self.kf.H = np.array([[1,0,0,0],
                              [0,1,0,0]])
        
        self.kf.P *= 500.                               # 초기 공분산
        self.kf.R = np.array([[0.8,0],                    
                              [0,0.8]])                   # 측정 노이즈
        self.kf.Q = np.array([[0.1, 0,   0,   0],  # x에 대한 예측 신뢰도
                              [0, 0.1,   0,   0],  # y에 대한 예측 신뢰도
                              [0,  0,  3,  0],  # v_x에 대한 예측 신뢰도
                              [0,  0,   0,  3]]) # v_y에 대한 예측 신뢰도 # 프로세스 노이즈
    
        self.kf.x[:2] = np.array([position[0], position[1]]).reshape(2, 1)
        self.kf.x[2:] = np.array([0, 0]).reshape(2, 1)
        self.id = KalmanPositionTracker.count
        KalmanPositionTracker.count += 1

        self.time_since_update = 0  # 업데이트 이후 경과한 시간
        self.history = []           # 상태들을 기록하는 리스트
        self.hits = 0               # 감지된 횟수
        self.hit_streak = 0         # 연속된 감지 횟수
        self.age = 0                # 객체의 프레임 수

        self.last_time = self.node.get_clock().now().nanoseconds * 1e-9
        self.current_time = self.node.get_clock().now().nanoseconds * 1e-9
        self.dt = 0.1

    def update(self, position):
        """
        관측된 position으로 상태를 업데이트하는 함수
        """
        self.time_since_update = 0
        self.history = []
        self.hits += 1
        self.hit_streak += 1
        self.kf.update(np.array([position[0], position[1]]))

    def predict(self):
        """
        상태 벡터를 예측하여 다음 프레임에서 객체의 위치를 추정하는 함수
        """
        self.current_time = self.node.get_clock().now().nanoseconds * 1e-9
        self.dt = self.current_time - self.last_time
        self.kf.F = np.array([[1, 0, self.dt, 0],
                              [0, 1, 0, self.dt],
                              [0, 0, 1,  0],
                              [0, 0, 0,  1]])
        self.last_time = self.current_time

        self.kf.predict()
        self.age += 1
        if (self.time_since_update > 0):
            self.hit_streak = 0
        self.time_since_update += 1
        self.history.append(self.kf.x)

        return self.history[-1]
    
    def get_state(self):
        """
        객체의 현재 추정값을 반환하는 함수
        """
        return self.kf.x

def quaternion_from_euler(roll, pitch, yaw):
    """
    Convert an Euler angle to a quaternion.
    
    Parameters:
    roll : float
        The roll (rotation around x-axis) angle in radians.
    pitch : float
        The pitch (rotation around y-axis) angle in radians.
    yaw : float
        The yaw (rotation around z-axis) angle in radians.
    
    Returns:
    tuple
        A tuple of 4 elements representing the quaternion (x, y, z, w).
    """
    qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
    qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2)
    qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2)
    qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
    
    return (qx, qy, qz, qw)


def main(args=None):
    rclpy.init(args=args)
    node = ObjectTracker()
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
