# observation_program/main.py

import sys
import rclpy
from rclpy.node import Node
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QGridLayout, QLabel,
    QPushButton, QTextEdit, QVBoxLayout, QHBoxLayout
)
from PyQt5.QtGui import QPixmap, QImage
from PyQt5.QtCore import Qt, QThread, pyqtSignal
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge
import numpy as np
import pyqtgraph as pg

from .main_camera_widget import MainCameraWidget
from .camera_feed_widget import CameraFeedWidget
from .lidar_view_widget import LidarViewWidget
from .data_analysis_widget import DataAnalysisWidget
from .data_fusion_widget import DataFusionWidget
from .control_interface_widget import ControlInterfaceWidget
from .mission_planning_widget import MissionPlanningWidget
from .status_panel_widget import StatusPanelWidget
from .history_notification_widget import HistoryNotificationWidget
from .ros2_listener import ROS2ListenerThread

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("관찰 프로그램")
        self.setFixedSize(800, 600)

        # 위젯 초기화
        self.init_widgets()
        # 레이아웃 설정
        self.setup_layout()
        # ROS2 노드 및 스레드 시작
        self.start_ros2_listener()

    def init_widgets(self):
        # 메뉴 바
        self.init_menu_bar()

        # 메인 카메라 피드 (멀티뷰 지원)
        self.main_camera_widget = MainCameraWidget()

        # 보조 카메라 피드
        self.sub_camera_widget = CameraFeedWidget("보조 카메라", 190, 145)

        # LiDAR 뷰
        self.lidar_view_widget = LidarViewWidget()

        # 데이터 분석 및 통계
        self.data_analysis_widget = DataAnalysisWidget()

        # 데이터 융합 디스플레이
        self.data_fusion_widget = DataFusionWidget()

        # 제어 인터페이스
        self.control_interface_widget = ControlInterfaceWidget()

        # 임무 계획 정보
        self.mission_planning_widget = MissionPlanningWidget()

        # 상태 정보 패널
        self.status_panel_widget = StatusPanelWidget()

        # 히스토리 및 알림 영역
        self.history_notifications_widget = HistoryNotificationWidget()

    def init_menu_bar(self):
        menubar = self.menuBar()
        menubar.addMenu('파일')
        menubar.addMenu('보기')
        menubar.addMenu('설정')
        menubar.addMenu('도움말')

    def setup_layout(self):
        central_widget = QWidget()
        grid = QGridLayout()

        # 위젯 배치
        grid.addWidget(self.main_camera_widget, 0, 0, 2, 2)
        grid.addWidget(self.sub_camera_widget, 0, 2)
        grid.addWidget(self.lidar_view_widget, 1, 2)
        grid.addWidget(self.data_analysis_widget, 0, 3)
        grid.addWidget(self.data_fusion_widget, 1, 3)
        grid.addWidget(self.control_interface_widget, 2, 0, 1, 2)
        grid.addWidget(self.mission_planning_widget, 2, 2, 1, 2)
        grid.addWidget(self.status_panel_widget, 3, 0, 1, 3)
        grid.addWidget(self.history_notifications_widget, 3, 3)

        central_widget.setLayout(grid)
        self.setCentralWidget(central_widget)

    def start_ros2_listener(self):
        self.ros_thread = ROS2ListenerThread()
        self.ros_thread.start()

        # 신호 연결
        self.ros_thread.listener.multi_camera_signal.connect(
            self.main_camera_widget.update_images
        )
        self.ros_thread.listener.sub_camera_signal.connect(
            self.sub_camera_widget.update_image
        )
        self.ros_thread.listener.lidar_signal.connect(
            self.lidar_view_widget.update_scan
        )
        # 상태 정보 업데이트 연결
        self.ros_thread.listener.status_signal.connect(
            self.status_panel_widget.update_status
        )
        # 히스토리 및 알림 업데이트 연결
        self.ros_thread.listener.history_signal.connect(
            self.history_notifications_widget.update_history
        )

    def closeEvent(self, event):
        self.ros_thread.stop()
        event.accept()

def main(args=None):
    rclpy.init(args=args)
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
