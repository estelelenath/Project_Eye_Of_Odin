class CameraFeedWidget(QLabel):
    def __init__(self, title, width, height):
        super().__init__()
        self.setFixedSize(width, height)
        self.setAlignment(Qt.AlignCenter)
        self.setText(title)
        self.setStyleSheet("border: 1px solid gray;")

    def update_image(self, qt_image):
        self.setPixmap(QPixmap.fromImage(qt_image))
