#!/usr/bin/env python3
"""ROS 2 wrapper for the transport-agnostic NTRIP client."""

import sys

import rclpy
from nmea_msgs.msg import Sentence
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rtcm_msgs.msg import Message

from ntrip_client.nmea_parser import NMEA_DEFAULT_MAX_LENGTH, NMEA_DEFAULT_MIN_LENGTH
from ntrip_client.ntrip_client import NTRIPClient


class NtripNode(Node):
    def __init__(self):
        super().__init__('ntrip_client')
        defaults = {
            'host': '127.0.0.1', 'port': 2101, 'mountpoint': 'mount',
            'ntrip_version': '', 'authenticate': False, 'username': '', 'password': '',
            'ssl': False, 'cert': '', 'key': '', 'ca_cert': '', 'rtcm_frame_id': 'odom',
            'nmea_max_length': NMEA_DEFAULT_MAX_LENGTH,
            'nmea_min_length': NMEA_DEFAULT_MIN_LENGTH,
            'reconnect_attempt_max': NTRIPClient.DEFAULT_RECONNECT_ATTEMPT_MAX,
            'reconnect_attempt_wait_seconds': NTRIPClient.DEFAULT_RECONNECT_ATEMPT_WAIT_SECONDS,
            'rtcm_timeout_seconds': NTRIPClient.DEFAULT_RTCM_TIMEOUT_SECONDS,
            'rtcm_chunk_size': 512,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        authenticate = self.get_parameter('authenticate').value
        username = self.get_parameter('username').value if authenticate else None
        password = self.get_parameter('password').value if authenticate else None
        if authenticate and (not username or not password):
            raise ValueError('authenticate=true requires non-empty username and password')

        self._frame_id = self.get_parameter('rtcm_frame_id').value
        self._chunk_size = self.get_parameter('rtcm_chunk_size').value
        if self._chunk_size <= 0:
            raise ValueError('rtcm_chunk_size must be greater than zero')

        self._client = NTRIPClient(
            host=self.get_parameter('host').value,
            port=self.get_parameter('port').value,
            mountpoint=self.get_parameter('mountpoint').value,
            ntrip_version=self.get_parameter('ntrip_version').value or None,
            username=username,
            password=password,
            logerr=self.get_logger().error,
            logwarn=self.get_logger().warning,
            loginfo=self.get_logger().info,
            logdebug=self.get_logger().debug,
        )
        self._client.ssl = self.get_parameter('ssl').value
        self._client.cert = self.get_parameter('cert').value or None
        self._client.key = self.get_parameter('key').value or None
        self._client.ca_cert = self.get_parameter('ca_cert').value or None
        self._client.nmea_parser.nmea_max_length = self.get_parameter('nmea_max_length').value
        self._client.nmea_parser.nmea_min_length = self.get_parameter('nmea_min_length').value
        self._client.reconnect_attempt_max = self.get_parameter('reconnect_attempt_max').value
        self._client.reconnect_attempt_wait_seconds = self.get_parameter('reconnect_attempt_wait_seconds').value
        self._client.rtcm_timeout_seconds = self.get_parameter('rtcm_timeout_seconds').value

        self._rtcm_pub = self.create_publisher(Message, 'rtcm', 10)
        self._nmea_sub = self.create_subscription(Sentence, 'nmea', self._on_nmea, 10)
        self._timer = None

    def connect(self):
        while rclpy.ok() and not self._client.connect():
            self.get_logger().error('Unable to connect to NTRIP server; retrying in 1 second')
            self._client.disconnect()
            rclpy.spin_once(self, timeout_sec=1.0)
        if rclpy.ok():
            self._timer = self.create_timer(0.1, self._publish_rtcm)

    def close(self):
        if self._timer is not None:
            self.destroy_timer(self._timer)
        self._client.shutdown()

    def _on_nmea(self, msg):
        self._client.send_nmea(msg.sentence)

    def _publish_rtcm(self):
        try:
            for raw_rtcm in self._client.recv_rtcm():
                for offset in range(0, len(raw_rtcm), self._chunk_size):
                    msg = Message()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = self._frame_id
                    msg.message = list(raw_rtcm[offset:offset + self._chunk_size])
                    self._rtcm_pub.publish(msg)
        except Exception as exc:
            self.get_logger().error(f'NTRIP receive failed: {exc}')


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = NtripNode()
        node.connect()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as exc:
        (node.get_logger().fatal if node else lambda msg: print(msg, file=sys.stderr))(str(exc))
        return 1
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
