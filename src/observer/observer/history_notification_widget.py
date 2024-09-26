class HistoryNotificationWidget(QTextEdit):
    def __init__(self):
        super().__init__()
        self.setFixedSize(250, 100)
        self.setStyleSheet("border: 1px solid gray; background-color: #f9f9f9;")
        self.setReadOnly(True)
        self.append("히스토리 및 알림")

    def update_history(self, message):
        self.append(message)
