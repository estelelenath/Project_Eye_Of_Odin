import sys
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QGridLayout, QLabel,
    QPushButton, QTextEdit, QVBoxLayout, QHBoxLayout, QMenuBar, QAction
)
from PyQt5.QtGui import QPixmap, QImage
from PyQt5.QtCore import Qt
import pyqtgraph as pg

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("관찰 프로그램")
        self.setFixedSize(800, 600)

        # 메뉴 바 생성
        self.init_menu_bar()

        # 중앙 위젯 설정
        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        # 전체 레이아웃
        grid = QGridLayout()
        central_widget.setLayout(grid)

        # 메인 카메라 피드 (멀티뷰 지원)
        self.main_camera_widget = self.create_main_camera_widget()
        grid.addWidget(self.main_camera_widget, 0, 0, 2, 2)

        # 보조 카메라 피드
        self.sub_camera_widget = self.create_label_widget("보조 카메라", 190, 145)
        grid.addWidget(self.sub_camera_widget, 0, 2)

        # LiDAR 뷰
        self.lidar_view_widget = self.create_label_widget("LiDAR 뷰", 190, 145)
        grid.addWidget(self.lidar_view_widget, 1, 2)

        # 데이터 분석 및 통계
        self.data_analysis_widget = self.create_label_widget("데이터 분석 및 통계", 180, 145)
        grid.addWidget(self.data_analysis_widget, 0, 3)

        # 데이터 융합 디스플레이
        self.data_fusion_widget = self.create_label_widget("데이터 융합 디스플레이", 180, 145)
        grid.addWidget(self.data_fusion_widget, 1, 3)

        # 제어 인터페이스
        self.control_interface_widget = self.create_control_interface_widget()
        grid.addWidget(self.control_interface_widget, 2, 0, 1, 2)

        # 임무 계획 정보
        self.mission_planning_widget = self.create_label_widget("임무 계획 정보", 380, 130)
        grid.addWidget(self.mission_planning_widget, 2, 2, 1, 2)

        # 상태 정보 패널
        self.status_panel_widget = self.create_status_panel_widget()
        grid.addWidget(self.status_panel_widget, 3, 0, 1, 3)

        # 히스토리 및 알림 영역
        self.history_notifications_widget = self.create_label_widget("히스토리 및 알림", 250, 100)
        grid.addWidget(self.history_notifications_widget, 3, 3)

    def init_menu_bar(self):
        menubar = self.menuBar()

        file_menu = menubar.addMenu('파일')
        view_menu = menubar.addMenu('보기')
        settings_menu = menubar.addMenu('설정')
        help_menu = menubar.addMenu('도움말')

        # 메뉴 액션 추가 (필요에 따라)
        # 예: file_menu.addAction(QAction('열기', self))

    def create_main_camera_widget(self):
        widget = QWidget()
        widget.setFixedSize(390, 300)
        layout = QGridLayout()
        widget.setLayout(layout)
        labels = []
        for i in range(4):  # 4개의 카메라 뷰
            label = QLabel(f"카메라 {i+1}")
            label.setAlignment(Qt.AlignCenter)
            label.setStyleSheet("border: 1px solid gray;")
            label.setFixedSize(190, 145)
            labels.append(label)
            row = i // 2
            col = i % 2
            layout.addWidget(label, row, col)
        return widget

    def create_label_widget(self, title, width, height):
        widget = QWidget()
        widget.setFixedSize(width, height)
        layout = QVBoxLayout()
        widget.setLayout(layout)
        label = QLabel(title)
        label.setAlignment(Qt.AlignCenter)
        label.setStyleSheet("border: 1px solid gray;")
        layout.addWidget(label)
        return widget

    def create_control_interface_widget(self):
        widget = QWidget()
        widget.setFixedSize(390, 130)
        widget.setStyleSheet("background-color: #e6e6e6; border: 1px solid gray;")
        layout = QGridLayout()
        widget.setLayout(layout)

        move_control_btn = QPushButton("이동 제어")
        camera_settings_btn = QPushButton("카메라 설정")
        lidar_settings_btn = QPushButton("LiDAR 설정")
        data_record_btn = QPushButton("데이터 기록")
        data_playback_btn = QPushButton("데이터 재생")

        layout.addWidget(move_control_btn, 0, 0)
        layout.addWidget(camera_settings_btn, 0, 1)
        layout.addWidget(lidar_settings_btn, 0, 2)
        layout.addWidget(data_record_btn, 1, 0)
        layout.addWidget(data_playback_btn, 1, 1)

        return widget

    def create_status_panel_widget(self):
        widget = QWidget()
        widget.setFixedSize(520, 100)
        widget.setStyleSheet("border: 1px solid gray;")
        layout = QGridLayout()
        widget.setLayout(layout)

        title_label = QLabel("상태 정보 패널")
        title_label.setAlignment(Qt.AlignLeft)
        layout.addWidget(title_label, 0, 0)

        battery_label = QLabel("배터리: 75%")
        temperature_label = QLabel("시스템 온도: 35°C")
        position_label = QLabel("현재 위치: X: 10.5, Y: 20.3")
        network_label = QLabel("네트워크: 양호")
        sensor_label = QLabel("센서 상태: 정상")

        layout.addWidget(battery_label, 1, 0)
        layout.addWidget(temperature_label, 2, 0)
        layout.addWidget(position_label, 3, 0)
        layout.addWidget(network_label, 1, 1)
        layout.addWidget(sensor_label, 2, 1)

        return widget

def main():
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()

