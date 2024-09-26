class ControlInterfaceWidget(QWidget):
    def __init__(self):
        super().__init__()
        self.setFixedSize(390, 130)
        self.setStyleSheet("background-color: #e6e6e6; border: 1px solid gray;")
        layout = QGridLayout()

        self.move_control_btn = QPushButton("이동 제어")
        self.camera_settings_btn = QPushButton("카메라 설정")
        self.lidar_settings_btn = QPushButton("LiDAR 설정")
        self.data_record_btn = QPushButton("데이터 기록")
        self.data_playback_btn = QPushButton("데이터 재생")

        layout.addWidget(self.move_control_btn, 0, 0)
        layout.addWidget(self.camera_settings_btn, 0, 1)
        layout.addWidget(self.lidar_settings_btn, 0, 2)
        layout.addWidget(self.data_record_btn, 1, 0)
        layout.addWidget(self.data_playback_btn, 1, 1)

        self.setLayout(layout)

        # 버튼 동작 연결
        self.move_control_btn.clicked.connect(self.move_control)
        self.camera_settings_btn.clicked.connect(self.camera_settings)
        self.lidar_settings_btn.clicked.connect(self.lidar_settings)
        self.data_record_btn.clicked.connect(self.data_record)
        self.data_playback_btn.clicked.connect(self.data_playback)

    def move_control(self):
        # 이동 제어 로직 구현
        pass

    def camera_settings(self):
        # 카메라 설정 로직 구현
        pass

    def lidar_settings(self):
        # LiDAR 설정 로직 구현
        pass

    def data_record(self):
        # 데이터 기록 로직 구현
        pass

    def data_playback(self):
        # 데이터 재생 로직 구현
        pass
