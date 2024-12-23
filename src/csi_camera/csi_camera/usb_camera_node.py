import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

def gstreamer_pipeline_usb(device_id=0, capture_width=1280, capture_height=720, framerate=30):
    """
    v4l2src를 사용하여 USB 카메라용 GStreamer 파이프라인 생성
    """
    return (
        f"v4l2src device=/dev/video{device_id} ! "
        f"video/x-raw, width=(int){capture_width}, height=(int){capture_height}, "
        f"framerate=(fraction){framerate}/1 ! "
        "videoconvert ! video/x-raw, format=BGR ! "
        "appsink"
    )

class USBCameraNode(Node):
    def __init__(self):
        super().__init__('usb_camera_node')
        # 파라미터 선언 및 가져오기
        self.declare_parameter('device_id', 0)
        self.declare_parameter('capture_width', 1280)
        self.declare_parameter('capture_height', 720)
        self.declare_parameter('framerate', 30)
        
        # 파라미터 가져오기			   
        self.device_id = self.get_parameter('device_id').value
        self.capture_width = self.get_parameter('capture_width').value
        self.capture_height = self.get_parameter('capture_height').value
        self.framerate = self.get_parameter('framerate').value
        # 퍼블리셔 생성 (QoS 설정)
        self.publisher_ = self.create_publisher(
            Image, 
            f'/cameras/usb_{self.device_id}/image_raw', 
            rclpy.qos.QoSProfile(depth=10, reliability=rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT)
        )

        # 카메라 초기화
        pipeline = gstreamer_pipeline_usb(
            device_id=self.device_id,
            capture_width=self.capture_width,
            capture_height=self.capture_height,
            framerate=self.framerate
        )
        self.cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
        
        if not self.cap.isOpened():
            self.get_logger().error(f"USB 카메라 {self.device_id}를 열 수 없습니다. 장치를 확인하세요.")
            return

        # CvBridge와 타이머 생성
        self.br = CvBridge()
        self.timer = self.create_timer(1.0/self.framerate, self.timer_callback)

        self.get_logger().info(
            f'USB 카메라 노드가 device_id: {self.device_id}로 초기화되었습니다.'
        )

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            self.publisher_.publish(self.br.cv2_to_imgmsg(frame, "bgr8"))
        else:
            self.get_logger().warning(f"USB 카메라 {self.device_id} 프레임 캡처 실패")

    def __del__(self):
        if hasattr(self, 'cap'):
            self.cap.release()

def main(args=None):
    rclpy.init(args=args)
    usb_camera_node = USBCameraNode()
    rclpy.spin(usb_camera_node)
    usb_camera_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
