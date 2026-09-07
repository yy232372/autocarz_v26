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

from autocarz.msg import CtrlCmd

class VehicleControler:
    """
    Manages the steering of the vehicle.
    """

    def __init__(self, node: Node):
        self.node = node
        self._subscriptions = []
        self.ctrl_pub = self.node.create_publisher(CtrlCmd, _topic(self.node, '/ctrlCmd'), CONTROL_QOS)
        self.ctrl_msg = CtrlCmd()

    def control(self, speed, brake, steering):
        self.ctrl_msg.velocity = float(speed)
        self.ctrl_msg.brake = float(brake)
        self.ctrl_msg.steering = float(steering)
        self.ctrl_pub.publish(self.ctrl_msg)
