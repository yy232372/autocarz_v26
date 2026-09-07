#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys

from ackermann_msgs.msg import AckermannDriveStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from autocarz.msg import CtrlCmd


CONTROL_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)


def clamp(value, min_value, max_value):
    return max(min_value, min(max_value, value))


class CarlaControlAdapter(Node):
    def __init__(self):
        super().__init__('carla_control_adapter')

        self.declare_parameter('ctrl_topic', '/ctrlCmd')
        self.declare_parameter('control_topic', '/carla/ego_vehicle/ackermann_control_cmd')
        self.declare_parameter('speed_scale', 1.0 / 3.6)
        self.declare_parameter('steering_scale', -1.0)
        self.declare_parameter('max_speed_mps', 1.0)
        self.declare_parameter('max_steering_rad', 0.6)
        self.declare_parameter('acceleration_mps2', 1.0)
        self.declare_parameter('brake_deceleration_mps2', 4.0)

        self.speed_scale = float(self.get_parameter('speed_scale').value)
        self.steering_scale = float(self.get_parameter('steering_scale').value)
        self.max_speed_mps = max(0.1, float(self.get_parameter('max_speed_mps').value))
        self.max_steering_rad = max(0.01, float(self.get_parameter('max_steering_rad').value))
        self.acceleration_mps2 = max(0.1, float(self.get_parameter('acceleration_mps2').value))
        self.brake_deceleration_mps2 = -max(0.1, float(self.get_parameter('brake_deceleration_mps2').value))

        self.control_pub = self.create_publisher(
            AckermannDriveStamped,
            str(self.get_parameter('control_topic').value),
            CONTROL_QOS,
        )
        self.create_subscription(
            CtrlCmd,
            str(self.get_parameter('ctrl_topic').value),
            self.ctrl_callback,
            CONTROL_QOS,
        )

    def ctrl_callback(self, msg):
        target_speed_mps = clamp(float(msg.velocity) * self.speed_scale, 0.0, self.max_speed_mps)
        steering_rad = clamp(float(msg.steering) * self.steering_scale, -self.max_steering_rad, self.max_steering_rad)
        brake_cmd = clamp(float(msg.brake) / 100.0, 0.0, 1.0)

        carla_msg = AckermannDriveStamped()
        carla_msg.header.stamp = self.get_clock().now().to_msg()
        carla_msg.header.frame_id = 'base_link'
        carla_msg.drive.speed = target_speed_mps
        carla_msg.drive.steering_angle = steering_rad
        carla_msg.drive.acceleration = (
            self.brake_deceleration_mps2 * brake_cmd
            if brake_cmd > 0.0
            else self.acceleration_mps2
        )
        carla_msg.drive.jerk = 0.0

        self.control_pub.publish(carla_msg)


def main(args=None):
    rclpy.init(args=args)
    node = CarlaControlAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
