class DataFusionWidget(QWidget):
    def __init__(self):
        super().__init__()
        self.setFixedSize(180, 145)
        self.setStyleSheet("border: 1px solid gray;")
        self.label = QLabel("데이터 융합 디스플레이")
        self.label.setAlignment(Qt.AlignCenter)
        self.layout = QVBoxLayout()
        self.layout.addWidget(self.label)
        self.setLayout(self.layout)
        # 필요시 추가적인 데이터 융합 로직 구현
