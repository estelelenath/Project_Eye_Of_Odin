class DataAnalysisWidget(QWidget):
    def __init__(self):
        super().__init__()
        self.setFixedSize(180, 145)
        self.setStyleSheet("border: 1px solid gray;")
        layout = QVBoxLayout()
        self.label = QLabel("데이터 분석 및 통계")
        self.label.setAlignment(Qt.AlignCenter)
        self.chart = pg.PlotWidget()
        self.chart.setBackground('w')
        layout.addWidget(self.label)
        layout.addWidget(self.chart)
        self.setLayout(layout)
        # 데이터 초기화
        self.data = []

    def update_data(self, new_data):
        self.data.append(new_data)
        self.chart.plot(self.data, clear=True)
