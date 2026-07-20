#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from ament_index_python.packages import get_package_share_directory
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

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

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class Logger:
    """
    A simple logging utility for ROS.
    """

    def __init__(self, node: Node):
        self.node = node
        self._subscriptions = []
        pass

    def log(self, message, messageType='info'):
        """
        Log a message using ROS logging system.
        """
        if messageType == 'info':
            self.node.get_logger().info(message)
        elif messageType == 'warn':
            self.node.get_logger().warning(message)
        elif messageType == 'error':
            self.node.get_logger().error(message)

class ConfigManager:
    """
    Manages configuration parameters retrieved from ROS parameters.
    """

    def __init__(self, node: Node):
        self.node = node
        self._subscriptions = []
        self.pkg = 'autocarz'
        self.pkgPath = get_package_share_directory(self.pkg)
        self.tf_broadcaster = TransformBroadcaster(self.node)
        pathIn_rel = _get_parameter(self.node, '/mode/pathIn')
        pathOut_rel = _get_parameter(self.node, '/mode/pathOut')
        self.pathInFile = self.pkgPath + '/' + pathIn_rel
        self.pathOutFile = self.pkgPath + '/' + pathOut_rel
        mode_name = _get_parameter(self.node, '/mode/name', 'unknown')
        self.node.get_logger().info('[MODE] %s | pathIn=%s pathOut=%s' % (mode_name, self.pathInFile, self.pathOutFile))
        self.pathInPub = self.node.create_publisher(Path, _topic(self.node, '/pathIn'), CONTROL_QOS)
        self.pathOutPub = self.node.create_publisher(Path, _topic(self.node, '/pathOut'), CONTROL_QOS)

    def get_path(self, pathType):
        """
        Return file paths for 'in' and 'out' path types.
        라인 수를 자동 산출해서 /mode/numGlobalPath* 로 publish.
        """
        if pathType == 'in':
            self.globalPathIn = self.readPath(self.pathInFile)
            n = len(self.globalPathIn.poses)
            _set_parameter(self.node, '/mode/numGlobalPathIn', n)
            self.node.get_logger().info('[MODE] numGlobalPathIn=%d' % (n,))
            return self.globalPathIn
        elif pathType == 'out':
            self.globalPathOut = self.readPath(self.pathOutFile)
            n = len(self.globalPathOut.poses)
            _set_parameter(self.node, '/mode/numGlobalPathOut', n)
            self.node.get_logger().info('[MODE] numGlobalPathOut=%d' % (n,))
            return self.globalPathOut

    def readPath(self, fileName):
        """
        Read path data from a file.
        """
        path = Path()
        path.header.frame_id = 'map'
        with open(fileName, 'r') as file:
            lines = file.readlines()
            for line in lines:
                data = line.split()
                pose = PoseStamped()
                pose.pose.position.x = float(data[0])
                pose.pose.position.y = float(data[1])
                pose.pose.position.z = 0.0
                pose.pose.orientation.w = float(data[2])
                path.poses.append(pose)
        return path

    def pub_path(self):
        self.pathInPub.publish(self.globalPathIn)
        self.pathOutPub.publish(self.globalPathOut)

    def broadcast(self, odom):
        transform = TransformStamped()
        transform.header.stamp = self.node.get_clock().now().to_msg()
        transform.header.frame_id = 'map'
        transform.child_frame_id = 'base_link'
        transform.transform.translation.x = odom.pose.pose.position.x
        transform.transform.translation.y = odom.pose.pose.position.y
        transform.transform.translation.z = odom.pose.pose.position.z
        transform.transform.rotation = odom.pose.pose.orientation
        self.tf_broadcaster.sendTransform(transform)
