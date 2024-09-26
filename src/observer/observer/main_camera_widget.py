# observation_program/main_camera_widget.py

from PyQt5.QtWidgets import QWidget, QLabel, QGridLayout
from PyQt5.QtGui import QPixmap
from PyQt5.QtCore import Qt

# observation_program/main.py (계속)

class MainCameraWidget(QWidget):
    def __init__(self):
        super().__init__()
        self.setFixedSize(390, 300)
        self.layout = QGridLayout()
        self.setLayout(self.layout)
        self.labels = []
        for i in range(4):  # 4개의 카메라 뷰
            label = QLabel(f"카메라 {i+1}")
            label.setAlignment(Qt.AlignCenter)
            label.setStyleSheet("border: 1px solid gray;")
            self.labels.append(label)
            row = i // 2
            col = i % 2
            self.layout.addWidget(label, row, col)

    def update_images(self, images):
        for i, image in enumerate(images):
            if i < len(self.labels):
                self.labels[i].setPixmap(QPixmap.fromImage(image))
