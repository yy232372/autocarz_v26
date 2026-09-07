#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import re
import signal
import sys
import threading

import rclpy
from PyQt5 import uic
from PyQt5.QtCore import QTimer
from PyQt5.QtGui import QColor
from PyQt5.QtWidgets import QApplication, QMainWindow
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Path
from std_msgs.msg import Float32, Bool

from autocarz.msg import CtrlCmd, WaypointInfo

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

def _get_parameter(node: Node, name: str, default):
    if not node.has_parameter(name):
        node.declare_parameter(name, default)
    return node.get_parameter(name).value

script_dir = os.path.dirname(os.path.abspath(__file__))
ui_path = os.path.join(script_dir, 'ERP42RQT.ui')
form_class = uic.loadUiType(ui_path)[0]

class ERP42RQTNode(Node):
    def __init__(self):
        super().__init__('erp42_rqt_node')
        self.declare_parameter('mode.name', 'unknown')

        pass

class WindowClass(QMainWindow, form_class) :
    def __init__(self, node: Node):
        super().__init__()
        self.node = node
        self._subscriptions = []
        self.setupUi(self)

        # ──────────────── Single GPS state variables (GPS Hz 계산용) ──────
        self.gps_msg_count = 0
        self.gps_previous_time = 0
        self.gps_current_time = 0
        self.gps_time_diff = 1

        self.gps_hz_color = QColor(0, 0, 0)

        self.velocity_stats = 0.0
        self.gps_status = 0
        self.gps_hz = 0

        # ──────────────── 신규 state variables ─────────────────────────
        self.vlp_front = 30.0
        self.vlp_side_front = 30.0
        self.vlp_side_back = -15.0
        self.livox_front = 30.0
        self.livox_side_front = 30.0
        self.livox_side_back = -15.0
        self.lane = 1
        self.indx_in = 0
        self.indx_out = 0
        self.last_waypoint_msg_time = None
        self.lfd_distance = 0.0
        self.curr_path_idx = 0
        self.is_line_changing = False
        self._last_color = {}    # setStyleSheet 캐싱용


        # ──────────────── Existing subscribers (속도 + 새 GPS) ─────────────
        self.sub_velocity = self.node.create_subscription(CtrlCmd, _topic(self.node, '/ctrlCmd'), self.velocity_status_text, CONTROL_QOS)
        self.sub_gps_status = self.node.create_subscription(NavSatFix, _topic(self.node, '/fix'), self.for_gps_status_text, qos_profile_sensor_data)

        # # ──────────────── 신규: mode rosparam (Phase 1 모드 전환 plan) ─────
        # self.mode_name = _get_parameter(self.node, 'mode.name', 'unknown')
        # self.is_pre = self.mode_name.startswith("pre")

        # # 모드 라벨 + 예선이면 2차선 칸 숨김 (위젯이 존재할 때만)
        # if hasattr(self, 'label_mode'):
        #     self.label_mode.setText(f"모드: {self.mode_name}")
        # if self.is_pre and hasattr(self, 'frame_lane2'):
        #     self.frame_lane2.setVisible(False)

        # ──────────────── mode rosparam (QTimer 폴링 방식) ───────────────
        self.mode_name = "unknown"
        self.is_pre = False
        # ROS 2에는 ROS 1식 전역 파라미터 서버가 없으므로 경로 길이는
        # /pathIn, /pathOut 토픽 콜백에서 직접 갱신한다.
        self.num_in = 0
        self.num_out = 0

        if hasattr(self, 'label_mode'):
            self.label_mode.setText("모드: unknown")

        # 1초마다 /mode/name 다시 읽음 (모드 변경 자동 감지)
        self.mode_timer = QTimer(self)
        self.mode_timer.timeout.connect(self.update_mode_label)
        self.mode_timer.start(1000)        

        # ──────────────── 신규: 장애물/추종 정보 구독자 ────────────────
        self._subscriptions.append(self.node.create_subscription(Float32, _topic(self.node, '/vlp/distance_front'), self.cb_vlp_front, CONTROL_QOS))
        self._subscriptions.append(self.node.create_subscription(Float32, _topic(self.node, '/vlp/distance_side_front'), self.cb_vlp_side_front, CONTROL_QOS))
        self._subscriptions.append(self.node.create_subscription(Float32, _topic(self.node, '/vlp/distance_side_back'), self.cb_vlp_side_back, CONTROL_QOS))
        self._subscriptions.append(self.node.create_subscription(Float32, _topic(self.node, '/livox/distance_front'), self.cb_livox_front, CONTROL_QOS))
        self._subscriptions.append(self.node.create_subscription(Float32, _topic(self.node, '/livox/distance_side_front'), self.cb_livox_side_front, CONTROL_QOS))
        self._subscriptions.append(self.node.create_subscription(Float32, _topic(self.node, '/livox/distance_side_back'), self.cb_livox_side_back, CONTROL_QOS))
        self._subscriptions.append(self.node.create_subscription(WaypointInfo, _topic(self.node, '/waypointInfo'), self.cb_waypoint_info, CONTROL_QOS))
        self._subscriptions.append(self.node.create_subscription(Float32, _topic(self.node, '/lfd_distance'), self.cb_lfd, CONTROL_QOS))
        self._subscriptions.append(self.node.create_subscription(Float32, _topic(self.node, '/curr_path_idx'), self.cb_path_idx, CONTROL_QOS))
        self._subscriptions.append(self.node.create_subscription(Bool, _topic(self.node, '/is_line_changing'), self.cb_line_changing, CONTROL_QOS))
        self._subscriptions.append(self.node.create_subscription(Path, _topic(self.node, '/pathIn'), self.cb_path_in, CONTROL_QOS))
        self._subscriptions.append(self.node.create_subscription(Path, _topic(self.node, '/pathOut'), self.cb_path_out, CONTROL_QOS))

        # ──────────────── Timer ────────────────────────────────────────
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_ui)
        self.timer.start(100)  # Update every 100ms


    def velocity_status_text(self, data):
        self.velocity_stats = data.velocity

    def for_gps_status_text(self, data):
        self.gps_status = data.status.status

        # init self.gps_previous_time
        if self.gps_previous_time:
            if self.gps_previous_time != data.header.stamp.sec:
                    self.gps_current_time = data.header.stamp.sec
        else :
            self.gps_previous_time = data.header.stamp.sec

        self.gps_msg_count += 1

        if self.gps_current_time != 0 and self.gps_current_time != self.gps_previous_time:
            self.gps_time_diff = self.gps_current_time - self.gps_previous_time
            self.gps_previous_time = self.gps_current_time

            # Hz 계산
            hz = self.gps_msg_count / self.gps_time_diff
            self.gps_hz = "{}".format(int(hz))
            self.gps_msg_count = 0

        # if gps status != 2 background become red
        if self.gps_status != 2:
             self.gps_hz_color = QColor(255, 0, 0)
        else:
             self.gps_hz_color = QColor(0, 0, 0)

    # ──────────────── 신규: 장애물/추종 정보 콜백 (단순 저장만) ──────────
    def cb_vlp_front(self, data):
        self.vlp_front = data.data

    def cb_vlp_side_front(self, data):
        self.vlp_side_front = data.data

    def cb_vlp_side_back(self, data):
        self.vlp_side_back = data.data

    def cb_livox_front(self, data):
        self.livox_front = data.data

    def cb_livox_side_front(self, data):
        self.livox_side_front = data.data

    def cb_livox_side_back(self, data):
        self.livox_side_back = data.data

    def cb_waypoint_info(self, data):
        self.lane = data.lane
        self.indx_in = data.indx_in
        self.indx_out = data.indx_out
        self.last_waypoint_msg_time = self.node.get_clock().now()    # alive 판정용

    def cb_lfd(self, data):
        self.lfd_distance = data.data

    def cb_path_idx(self, data):
        self.curr_path_idx = int(data.data)

    def cb_path_in(self, data):
        self.num_in = len(data.poses)

    def cb_path_out(self, data):
        self.num_out = len(data.poses)

    def cb_line_changing(self, data):
        self.is_line_changing = data.data

    def update_mode_label(self):
        try:
            new_mode = _get_parameter(self.node, 'mode.name', 'unknown')
        except Exception:
            return    # ROS 2 통신 일시 이상 등 — 다음 cycle 에 재시도
        
        if new_mode == self.mode_name:
            return
        self.mode_name = new_mode
        self.is_pre = new_mode.startswith("pre")
        if hasattr(self, 'label_mode'):
            self.label_mode.setText(f"모드: {new_mode}")
        if hasattr(self, 'frame_lane2'):
            self.frame_lane2.setVisible(not self.is_pre)    

    # def update_mode_label(self):
    #     is_main_alive = False
    #     if self.last_waypoint_msg_time is not None:
    #         dt = (self.node.get_clock().now() - self.last_waypoint_msg_time).nanoseconds * 1e-9
    #         is_main_alive = (dt < 1.0)

    #     if is_main_alive:
    #         try:
    #             new_mode = _get_parameter(self.node, 'mode.name', 'unknown')
    #         except Exception:
    #             return
    #     else:
    #         new_mode = "unknown"
    #     ...

    def update_ui(self):
        # ─── Existing 위젯 (속도, GPS) ───
        self.lcdNumber.display(self.velocity_stats)
        self.lcdNumber_3.display(self.gps_status)
        self.lcdNumber_2.display(self.gps_hz)
        self.lcdNumber_5.display(0)
        self.lcdNumber_4.display(0)
        self.lcdNumber_3.setStyleSheet(f"QLCDNumber {{background-color: {self.gps_hz_color.name()};border: 2px solid white;}}")
        self.lcdNumber_5.setStyleSheet("QLCDNumber {background-color: black;border: 2px solid white;}")
        self.progressBar.setValue(int(self.velocity_stats))
        self.progressBar.setMaximum(20)

        # ─── 신규 위젯 (Step 1 Designer 작업 후 활성) ───
        if hasattr(self, 'label_lfd'):
            self._update_new_ui()

    # ──────────────── 신규: 차선/장애물/추종 영역 업데이트 ──────────────
    def _update_new_ui(self):
        # 1. planner alive 판정 (waypointInfo 1초 이내 수신 = alive)
        is_main_alive = False
        if self.last_waypoint_msg_time is not None:
            dt = (self.node.get_clock().now() - self.last_waypoint_msg_time).nanoseconds * 1e-9
            is_main_alive = (dt < 1.0)

        # 2. LFD / path_idx 라벨 (모드 무관)
        self.label_lfd.setText("LFD: %.1fm" % self.lfd_distance)
        self.label_path_idx.setText("path_idx: %d" % self.curr_path_idx)

        # 3. planner 꺼짐 → 차 이모지/waypoint 모두 비움
        if not is_main_alive:
            self.label_car_lane1.setText("")
            self.label_car_lane2.setText("")
            self.label_lane1_waypoint.setText("")
            self.label_lane2_waypoint.setText("")
            return

        # 4. 차 이모지 (line changing 중이면 200ms 단위 깜빡)
        emoji = "🚗"
        if self.is_line_changing:
            if int(self.node.get_clock().now().nanoseconds * 1e-9 * 5) % 2 == 0:
                emoji = ""

        # 5. lane 따라 좌/우 swap + 거리값 LCD swap
        # 예선 모드는 lane 토글되어도 항상 1차선 칸 사용 (frame_lane2 는 setVisible(False))
        use_lane1_as_primary = (self.lane == 1) or self.is_pre

        if use_lane1_as_primary:
            self.label_car_lane1.setText(emoji)
            self.label_car_lane2.setText("")
            self.label_lane1_waypoint.setText("%d / %d" % (self.indx_in, self.num_in))
            self.label_lane2_waypoint.setText("")

            # 1차선 칸 = 내 차선, 2차선 칸 = 옆 차선 (예선이면 frame_lane2 hidden)
            self._set_lcd(self.lcd_lane1_front_vlp, self.vlp_front)
            self._set_lcd(self.lcd_lane1_front_livox, self.livox_front)
            self._set_lcd(self.lcd_lane1_side_front_vlp, None)    # 1차선 측방은 비움
            self._set_lcd(self.lcd_lane1_side_front_livox, None)
            self._set_lcd(self.lcd_lane1_side_back_vlp, None)
            self._set_lcd(self.lcd_lane1_side_back_livox, None)
            if not self.is_pre:
                self._set_lcd(self.lcd_lane2_front_vlp, None)        # 2차선 전방 비움
                self._set_lcd(self.lcd_lane2_front_livox, None)
                self._set_lcd(self.lcd_lane2_side_front_vlp, self.vlp_side_front)
                self._set_lcd(self.lcd_lane2_side_front_livox, self.livox_side_front)
                self._set_lcd(self.lcd_lane2_side_back_vlp, abs(self.vlp_side_back))
                self._set_lcd(self.lcd_lane2_side_back_livox, abs(self.livox_side_back))
        else:  # lane == 2 (본선만 도달)
            self.label_car_lane1.setText("")
            self.label_car_lane2.setText(emoji)
            self.label_lane1_waypoint.setText("")
            self.label_lane2_waypoint.setText("%d / %d" % (self.indx_out, self.num_out))

            self._set_lcd(self.lcd_lane2_front_vlp, self.vlp_front)
            self._set_lcd(self.lcd_lane2_front_livox, self.livox_front)
            self._set_lcd(self.lcd_lane2_side_front_vlp, None)
            self._set_lcd(self.lcd_lane2_side_front_livox, None)
            self._set_lcd(self.lcd_lane2_side_back_vlp, None)
            self._set_lcd(self.lcd_lane2_side_back_livox, None)
            self._set_lcd(self.lcd_lane1_front_vlp, None)
            self._set_lcd(self.lcd_lane1_front_livox, None)
            self._set_lcd(self.lcd_lane1_side_front_vlp, self.vlp_side_front)
            self._set_lcd(self.lcd_lane1_side_front_livox, self.livox_side_front)
            self._set_lcd(self.lcd_lane1_side_back_vlp, abs(self.vlp_side_back))
            self._set_lcd(self.lcd_lane1_side_back_livox, abs(self.livox_side_back))

    def _set_lcd(self, lcd, value):
        """거리 LCD 에 값 + 색상 설정.
        - 테두리 색상으로 VLP(#FFD700 노랑) / Livox(#19FFF0 청록) 구분
        - 배경 색상으로 거리 위험도 구분 (빨/노/초/회)
        - value=None 이면 비활성 (0 + 회색 배경)
        """
        # VLP/Livox 센서 구분 (objectName 기반)
        sensor_color = "#FFD700" if "_vlp" in lcd.objectName() else "#19FFF0"

        if value is None:
            lcd.display(0)
            bg_color = "#404040"    # 비활성: 어두운 회색
        else:
            lcd.display(value)
            if value < 5.0:
                bg_color = "#cc0000"        # 빨강 (위험)
            elif value < 15.0:
                bg_color = "#cc8800"        # 주황 (경고) — 노랑 대신 주황으로 (테두리 노랑과 분리)
            else:
                bg_color = "#006600"        # 어두운 초록 (안전)

        new_style = "QLCDNumber { background-color: %s; border: 2px solid %s; }" % (bg_color, sensor_color)
        wid = id(lcd)
        if self._last_color.get(wid) != new_style:
            lcd.setStyleSheet(new_style)
            self._last_color[wid] = new_style



def main(args=None):
    rclpy.init(args=args)
    node = ERP42RQTNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    app = QApplication(sys.argv)
    window = WindowClass(node)
    window.show()
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    try:
        exit_code = app.exec_()
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=1.0)
    raise SystemExit(exit_code)

if __name__ == '__main__':
    main()
