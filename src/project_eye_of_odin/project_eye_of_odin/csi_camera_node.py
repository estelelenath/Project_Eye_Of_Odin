import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

def gstreamer_pipeline(
    sensor_id=0,
    capture_width=1280,
    capture_height=720,
    framerate=30,
):
    return (
        f"nvarguscamerasrc sensor-id={sensor_id} ! "
        f"video/x-raw(memory:NVMM), width=(int){capture_width}, height=(int){capture_height}, "
        f"framerate=(fraction){framerate}/1 ! "
        f"nvvidconv ! video/x-raw, format=(string)BGRx ! "
        "videoconvert ! video/x-raw, format=(string)BGR ! appsink"
    )

class CSICameraNode(Node):
    def __init__(self):
        super().__init__('csi_camera_node')
        self.declare_parameter('sensor_id', 0)
        
        self.sensor_id = self.get_parameter('sensor_id').value
        self.framerate = 30  # 고정 프레임레이트
        self.publisher_ = self.create_publisher(Image, f'camera{self.sensor_id}/image_raw', 10)
        self.timer = self.create_timer(1.0/self.framerate, self.timer_callback)
        self.cap = cv2.VideoCapture(gstreamer_pipeline(sensor_id=self.sensor_id, framerate=self.framerate), cv2.CAP_GSTREAMER)
        self.br = CvBridge()
        self.get_logger().info(f'CSI Camera Node initialized with sensor_id: {self.sensor_id}')

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            self.publisher_.publish(self.br.cv2_to_imgmsg(frame, "bgr8"))
        else:
            self.get_logger().warning("Failed to capture image")

    def __del__(self):
        self.cap.release()

def main(args=None):
    rclpy.init(args=args)
    csi_camera_node = CSICameraNode()
    rclpy.spin(csi_camera_node)
    csi_camera_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
