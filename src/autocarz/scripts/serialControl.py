#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import re
import threading

import numpy as np
import rclpy
import serial
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from autocarz.msg import CtrlCmd, Erp42State

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


class SerialControlNode(Node):
    def __init__(self):
        super().__init__('serial_control')

        self.steer_scale = 100.0 * 180.0 / math.pi
        self.speed_scale = 50.0
        self.estop = 0
        self.gear = 0
        self.speed = 0
        self.steer = 0
        self.brake = 0

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)
        self.serial_port = str(self.get_parameter('port').value)
        self.serial_baud = int(self.get_parameter('baud').value)

        self.ctrl_cmd_sub = self.create_subscription(
            CtrlCmd,
            _topic(self, '/ctrlCmd'),
            self.get_ctrl_cmd,
            CONTROL_QOS,
        )
        self.erp42_state_pub = self.create_publisher(
            Erp42State,
            _topic(self, '/erp42/state'),
            CONTROL_QOS,
        )

        self.serial = serial.Serial(self.serial_port, self.serial_baud)
        self.alive = 0
        self.rate = self.create_rate(30.0)

    def get_ctrl_cmd(self, data: CtrlCmd) -> None:
        self.speed = int(np.clip(int(data.velocity * self.speed_scale), 0, 200))
        self.steer = int(np.clip(int(data.steering * self.steer_scale * -1.0), -1300, 1300))
        self.brake = int(np.clip(int(data.brake), 0, 100))
        self.get_logger().info(
            'speed: %.2f, steer: %.2f, brake: %d'
            % (self.speed / self.speed_scale, self.steer / 71.0, self.brake)
        )

    def run(self) -> None:
        state = Erp42State()
        try:
            while rclpy.ok():
                data = []
                while rclpy.ok() and self.serial.read() != b'\n':
                    pass
                if not rclpy.ok():
                    break

                for _ in range(18):
                    raw = self.serial.read()
                    if not raw:
                        raise RuntimeError('ERP42 직렬 데이터 수신이 중단되었습니다.')
                    data.append(raw[0])

                state.velocity = (
                    ((data[6] & 0xFF) + ((data[7] << 8) & 0xFF00))
                    * 583.0
                    * math.pi
                    / 60000.0
                    * 3.6
                )
                steer_temp = (data[8] & 0xFF) + ((data[9] << 8) & 0xFF00)
                if steer_temp > 50000:
                    steer_temp -= 65536
                state.steering = -steer_temp / 100.0

                speed_l = self.speed & 0xFF
                speed_h = (self.speed & 0xFF00) >> 8
                steer_l = self.steer & 0xFF
                steer_h = (self.steer & 0xFF00) >> 8

                self.serial.write(b'STX')
                self.serial.write(
                    bytearray(
                        [
                            1,
                            self.estop,
                            self.gear,
                            speed_h,
                            speed_l,
                            steer_h,
                            steer_l,
                            self.brake,
                            self.alive,
                        ]
                    )
                )
                self.serial.write(b'\r\n')

                self.alive = (self.alive + 1) % 256
                self.erp42_state_pub.publish(state)
                self.rate.sleep()
        finally:
            if self.serial.is_open:
                self.serial.close()


def main(args=None):
    rclpy.init(args=args)
    node = SerialControlNode()
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
