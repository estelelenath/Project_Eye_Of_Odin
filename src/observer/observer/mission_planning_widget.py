class MissionPlanningWidget(QWidget):
    def __init__(self):
        super().__init__()
        self.setFixedSize(380, 130)
        self.setStyleSheet("border: 1px solid gray;")
        layout = QVBoxLayout()
        self.label = QLabel("임무 계획 정보")
        self.label.setAlignment(Qt.AlignCenter)
        self.map_view = QLabel()
        self.map_view.setFixedSize(360, 90)
        self.map_view.setStyleSheet("background-color: #f9f9f9; border: 1px solid gray;")
        layout.addWidget(self.label)
        layout.addWidget(self.map_view)
        self.setLayout(layout)
