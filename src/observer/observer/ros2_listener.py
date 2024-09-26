# observation_program/ros2_listener.py

import rclpy
from rclpy.node import Node
from PyQt5.QtCore import QObject, pyqtSignal, QThread
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge
import numpy as np
import cv2


# observation_program/main.py (계속)

class ROS2Listener(Node):
    multi_camera_signal = pyqtSignal(list)
    sub_camera_signal = pyqtSignal(QImage)
    lidar_signal = pyqtSignal(np.ndarray, np.ndarray)
    status_signal = pyqtSignal(dict)
    history_signal = pyqtSignal(str)

    def __init__(self):
        super().__init__('observation_program_listener')
        self.bridge = CvBridge()

        # 멀티 카메라 토픽 구독 (예: /camera0/image_raw, /camera1/image_raw, ...)
        self.camera_topics = ['/camera0/image_raw', '/camera1/image_raw', '/camera2/image_raw', '/camera3/image_raw']
        self.camera_subscriptions = []
        for topic in self.camera_topics:
            sub = self.create_subscription(Image, topic, self.multi_camera_callback, 10)
            self.camera_subscriptions.append(sub)

        # 보조 카메라 구독
        self.sub_camera_sub = self.create_subscription(Image, '/sub_camera/image_raw', self.sub_camera_callback, 10)

        # LiDAR 데이터 구독
        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)

        # 상태 정보 구독 (예: 배터리 상태, 온도 등)
        # self.status_sub = self.create_subscription(StatusMsg, '/status', self.status_callback, 10)

    def multi_camera_callback(self, msg):
        # 각 카메라 이미지 수신 및 업데이트
        # 구현 방법: 각 토픽별로 이미지를 저장하고, 모두 수신되면 업데이트
        pass

    def sub_camera_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        qt_image = self.convert_cv_to_qt(cv_image)
        self.sub_camera_signal.emit(qt_image)

    def lidar_callback(self, msg):
        angles = np.arange(msg.angle_min, msg.angle_max, msg.angle_increment)
        ranges = np.array(msg.ranges)
        self.lidar_signal.emit(angles, ranges)

    def convert_cv_to_qt(self, cv_image):
        rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_image.shape
        bytes_per_line = ch * w
        qt_image = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888)
        return qt_image

class ROS2ListenerThread(QThread):
    def __init__(self):
        super().__init__()
        self.listener = ROS2Listener()

    def run(self):
        rclpy.spin(self.listener)

    def stop(self):
        self.listener.destroy_node()
        rclpy.shutdown()
        self.quit()

