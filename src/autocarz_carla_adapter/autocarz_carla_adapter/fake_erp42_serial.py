#!/usr/bin/env python3

import argparse
import errno
import os
import pty
import select
import signal
import sys
import time


WHEEL_TO_KMH = 583.0 * 3.141592653589793 / 60000.0 * 3.6


class FakeErp42Serial:
    def __init__(self, link_path, rate_hz):
        self.link_path = link_path
        self.period = 1.0 / rate_hz
        self.master_fd = None
        self.running = True
        self.cmd_speed_kmh = 0.0
        self.cmd_steer_deg = 0.0
        self.rx_buffer = bytearray()

    def setup(self):
        self.master_fd, slave_fd = pty.openpty()
        slave_name = os.ttyname(slave_fd)
        os.close(slave_fd)
        os.set_blocking(self.master_fd, False)

        if os.path.lexists(self.link_path):
            if not os.path.islink(self.link_path):
                raise RuntimeError('%s exists and is not a symlink.' % self.link_path)
            os.unlink(self.link_path)
        os.symlink(slave_name, self.link_path)
        print('fake ERP42 serial: %s -> %s' % (self.link_path, slave_name), flush=True)

    def close(self):
        try:
            if os.path.islink(self.link_path):
                os.unlink(self.link_path)
        finally:
            if self.master_fd is not None:
                os.close(self.master_fd)

    def stop(self, *_):
        self.running = False

    def read_commands(self):
        while True:
            readable, _, _ = select.select([self.master_fd], [], [], 0.0)
            if not readable:
                return
            try:
                data = os.read(self.master_fd, 1024)
            except OSError as exc:
                if exc.errno in (errno.EAGAIN, errno.EWOULDBLOCK, errno.EIO):
                    return
                raise
            if not data:
                return
            self.rx_buffer.extend(data)

            while True:
                start = self.rx_buffer.find(b'STX')
                if start < 0:
                    self.rx_buffer.clear()
                    break
                if len(self.rx_buffer) < start + 14:
                    if start > 0:
                        del self.rx_buffer[:start]
                    break
                frame = self.rx_buffer[start:start + 14]
                del self.rx_buffer[:start + 14]
                self.parse_frame(frame)

    def parse_frame(self, frame):
        payload = frame[3:12]
        speed_raw = ((payload[3] & 0xFF) << 8) | (payload[4] & 0xFF)
        steer_raw = ((payload[5] & 0xFF) << 8) | (payload[6] & 0xFF)
        if steer_raw >= 32768:
            steer_raw -= 65536

        self.cmd_speed_kmh = speed_raw / 50.0
        self.cmd_steer_deg = -steer_raw / 100.0

    def state_packet(self):
        speed_raw = int(max(0, min(65535, round(self.cmd_speed_kmh / WHEEL_TO_KMH))))
        steer_raw = int(max(-32768, min(32767, round(-self.cmd_steer_deg * 100.0))))
        if steer_raw < 0:
            steer_raw += 65536

        data = bytearray(18)
        data[6] = speed_raw & 0xFF
        data[7] = (speed_raw >> 8) & 0xFF
        data[8] = steer_raw & 0xFF
        data[9] = (steer_raw >> 8) & 0xFF
        return b'\n' + bytes(data)

    def run(self):
        signal.signal(signal.SIGINT, self.stop)
        signal.signal(signal.SIGTERM, self.stop)
        self.setup()
        try:
            while self.running:
                started = time.monotonic()
                self.read_commands()
                try:
                    os.write(self.master_fd, self.state_packet())
                except OSError as exc:
                    if exc.errno != errno.EIO:
                        raise
                elapsed = time.monotonic() - started
                time.sleep(max(0.0, self.period - elapsed))
        finally:
            self.close()


def main():
    parser = argparse.ArgumentParser(description='Create a fake /dev/ttyUSB0 for autocarz serialControl.py.')
    parser.add_argument('--link', default='/dev/ttyUSB0')
    parser.add_argument('--rate', default=30.0, type=float)
    args = parser.parse_args()

    try:
        FakeErp42Serial(args.link, args.rate).run()
    except PermissionError:
        print('permission denied creating %s; run this command with sudo.' % args.link, file=sys.stderr)
        return 1
    except Exception as exc:
        print(str(exc), file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
