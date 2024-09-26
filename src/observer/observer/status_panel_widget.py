class StatusPanelWidget(QWidget):
    def __init__(self):
        super().__init__()
        self.setFixedSize(520, 100)
        self.setStyleSheet("border: 1px solid gray;")
        layout = QGridLayout()
        self.label = QLabel("상태 정보 패널")
        self.label.setAlignment(Qt.AlignLeft)
        layout.addWidget(self.label, 0, 0)

        self.battery_label = QLabel("배터리: 75%")
        self.temperature_label = QLabel("시스템 온도: 35°C")
        self.position_label = QLabel("현재 위치: X: 0.0, Y: 0.0")
        self.network_label = QLabel("네트워크: 양호")
        self.sensor_label = QLabel("센서 상태: 정상")

        layout.addWidget(self.battery_label, 1, 0)
        layout.addWidget(self.temperature_label, 2, 0)
        layout.addWidget(self.position_label, 3, 0)
        layout.addWidget(self.network_label, 1, 1)
        layout.addWidget(self.sensor_label, 2, 1)

        self.setLayout(layout)

    def update_status(self, status_dict):
        self.battery_label.setText(f"배터리: {status_dict['battery']}%")
        self.temperature_label.setText(f"시스템 온도: {status_dict['temperature']}°C")
        self.position_label.setText(f"현재 위치: X: {status_dict['position_x']}, Y: {status_dict['position_y']}")
        self.network_label.setText(f"네트워크: {status_dict['network']}")
        self.sensor_label.setText(f"센서 상태: {status_dict['sensor']}")

