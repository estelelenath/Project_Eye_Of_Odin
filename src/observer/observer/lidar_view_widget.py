# observation_program/lidar_view_widget.py

import pyqtgraph as pg
from PyQt5.QtWidgets import QWidget
import numpy as np

class LidarViewWidget(pg.PlotWidget):
    def __init__(self):
        super().__init__()
        self.setFixedSize(190, 145)
        self.setStyleSheet("border: 1px solid gray;")
        self.showGrid(x=True, y=True)
        self.setXRange(-10, 10)
        self.setYRange(-10, 10)
        self.plot_item = self.getPlotItem()
        self.plot_item.setTitle("LiDAR 뷰")
        self.scatter = pg.ScatterPlotItem(size=5, brush=pg.mkBrush(255, 0, 0, 120))
        self.addItem(self.scatter)

    def update_scan(self, angles, ranges):
        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)
        points = [{'pos': [x[i], y[i]]} for i in range(len(x))]
        self.scatter.setData(points)
