#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import re

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32

from autocarz.msg import CtrlCmd, Erp42State

try:
    from rviz_2d_overlay_msgs.msg import OverlayText
except ImportError as exc:
    raise ImportError(
        'ROS 2용 rviz_2d_overlay_msgs가 필요합니다. '
        '설치되지 않았다면 jsk_visualization.py를 실행할 수 없습니다.'
    ) from exc

CONTROL_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)


def _topic(node: Node, default_topic: str) -> str:
    slug = re.sub(r'[^A-Za-z0-9_]', '_', default_topic.strip('/'))
    slug = re.sub(r'_+', '_', slug).strip('_') or 'root'
    parameter_name = f'topics.{slug}'
    if not node.has_parameter(parameter_name):
        node.declare_parameter(parameter_name, default_topic)
    return str(node.get_parameter(parameter_name).value)


class TopicCtrlCmdGpsNode(Node):
    def __init__(self):
        super().__init__('jsk_visualization')

        self.sub_velocity = self.create_subscription(
            Erp42State,
            _topic(self, '/erp42/state'),
            self.velocity_pie_chart,
            CONTROL_QOS,
        )
        self.sub_steering = self.create_subscription(
            CtrlCmd,
            _topic(self, '/ctrlCmd'),
            self.steering_pie_chart,
            CONTROL_QOS,
        )
        self.sub_ublox1_status = self.create_subscription(
            NavSatFix,
            _topic(self, '/ublox1/fix'),
            self.for_reargps_status_text,
            qos_profile_sensor_data,
        )
        self.sub_ublox2_status = self.create_subscription(
            NavSatFix,
            _topic(self, '/ublox2/fix'),
            self.for_frontgps_status_text,
            qos_profile_sensor_data,
        )
        self.sub_gps_alarm1 = self.create_subscription(
            NavSatFix,
            _topic(self, '/ublox1/fix'),
            self.for_reargps_alarm_text,
            qos_profile_sensor_data,
        )
        self.sub_gps_alarm2 = self.create_subscription(
            NavSatFix,
            _topic(self, '/ublox2/fix'),
            self.for_frontgps_alarm_text,
            qos_profile_sensor_data,
        )

        self.pub_velocity = self.create_publisher(
            Float32, _topic(self, '/erp42_velocity'), CONTROL_QOS
        )
        self.pub_steering = self.create_publisher(
            Float32, _topic(self, '/erp42_steering'), CONTROL_QOS
        )
        self.pub_rear_gps = self.create_publisher(
            OverlayText, _topic(self, '/erp42_reargps_status'), CONTROL_QOS
        )
        self.pub_front_gps = self.create_publisher(
            OverlayText, _topic(self, '/erp42_frontgps_status'), CONTROL_QOS
        )
        self.pub_gps1_alarm = self.create_publisher(
            OverlayText, _topic(self, '/erp42_gps1_alarm'), CONTROL_QOS
        )
        self.pub_gps2_alarm = self.create_publisher(
            OverlayText, _topic(self, '/erp42_gps2_alarm'), CONTROL_QOS
        )
        self.pub_gps1_hz = self.create_publisher(
            OverlayText, _topic(self, '/erp42_gps1_hz'), CONTROL_QOS
        )
        self.pub_gps2_hz = self.create_publisher(
            OverlayText, _topic(self, '/erp42_gps2_hz'), CONTROL_QOS
        )

        self.rear_msg_count = 0
        self.rear_previous_time = 0
        self.rear_current_time = 0
        self.rear_time_diff = 1
        self.front_msg_count = 0
        self.front_previous_time = 0
        self.front_current_time = 0
        self.front_time_diff = 1

    def velocity_pie_chart(self, data: Erp42State) -> None:
        self.pub_velocity.publish(Float32(data=float(data.velocity)))

    def steering_pie_chart(self, data: CtrlCmd) -> None:
        self.pub_steering.publish(Float32(data=float(data.steering)))

    def for_reargps_status_text(self, data: NavSatFix) -> None:
        msg = OverlayText()
        msg.text = '[ublox1(Rear GPS)]\n' + str(data.status)
        self.pub_rear_gps.publish(msg)

        hz_msg = OverlayText()
        if self.rear_previous_time:
            if self.rear_previous_time != data.header.stamp.sec:
                self.rear_current_time = data.header.stamp.sec
        else:
            self.rear_previous_time = data.header.stamp.sec

        self.rear_msg_count += 1
        if self.rear_current_time != 0 and self.rear_current_time != self.rear_previous_time:
            self.rear_time_diff = self.rear_current_time - self.rear_previous_time
            self.rear_previous_time = self.rear_current_time
            hz = self.rear_msg_count / self.rear_time_diff
            hz_msg.text = 'GPS1: {} Hz'.format(int(hz))
            self.pub_gps1_hz.publish(hz_msg)
            self.rear_msg_count = 0

    def for_frontgps_status_text(self, data: NavSatFix) -> None:
        msg = OverlayText()
        msg.text = '[ublox2(Front GPS)]\n' + str(data.status)
        self.pub_front_gps.publish(msg)

        hz_msg = OverlayText()
        if self.front_previous_time:
            if self.front_previous_time != data.header.stamp.sec:
                self.front_current_time = data.header.stamp.sec
        else:
            self.front_previous_time = data.header.stamp.sec

        self.front_msg_count += 1
        if self.front_current_time != 0 and self.front_current_time != self.front_previous_time:
            self.front_time_diff = self.front_current_time - self.front_previous_time
            self.front_previous_time = self.front_current_time
            hz = self.front_msg_count / self.front_time_diff
            hz_msg.text = 'GPS2: {} Hz'.format(int(hz))
            self.pub_gps2_hz.publish(hz_msg)
            self.front_msg_count = 0

    def for_reargps_alarm_text(self, data: NavSatFix) -> None:
        msg = OverlayText()
        msg.text = 'X' if data.status.status != 2 else ''
        self.pub_gps1_alarm.publish(msg)

    def for_frontgps_alarm_text(self, data: NavSatFix) -> None:
        msg = OverlayText()
        msg.text = 'X' if data.status.status != 2 else ''
        self.pub_gps2_alarm.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TopicCtrlCmdGpsNode()
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
