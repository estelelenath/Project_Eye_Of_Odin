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
        "image/jpeg, "
        f"width=(int){capture_width}, height=(int){capture_height}, "
        f"framerate=(fraction){framerate}/1 ! "
        "jpegdec ! "
        "videoflip method=rotate-180 ! "  # 180도 회전을 위한 videoflip 추가
        "videoconvert ! "
        "video/x-raw, format=BGR ! "
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
        
        self.device_id = self.get_parameter('device_id').value
        self.capture_width = self.get_parameter('capture_width').value
        self.capture_height = self.get_parameter('capture_height').value
        self.framerate = self.get_parameter('framerate').value
        # QoS 설정 수정
        qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.QoSReliabilityPolicy.RELIABLE,
            durability=rclpy.qos.QoSDurabilityPolicy.VOLATILE,
            history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # 퍼블리셔 생성
        self.publisher_ = self.create_publisher(
            Image, 
            f'/cameras/usb_{self.device_id}/image_raw', 
            qos_profile
        )
        
        # 카메라 초기화 시도
        try:
            pipeline = gstreamer_pipeline_usb(
                device_id=self.device_id,
                capture_width=self.capture_width,
                capture_height=self.capture_height,
                framerate=self.framerate
            )
            self.get_logger().info(f'파이프라인: {pipeline}')
            
            self.cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
            if not self.cap.isOpened():
                # GStreamer 실패시 기본 V4L2 시도
                self.cap = cv2.VideoCapture(f"/dev/video{self.device_id}")
                if not self.cap.isOpened():
                    raise RuntimeError(f"카메라를 열 수 없습니다: /dev/video{self.device_id}")
                
                # 카메라 속성 설정
                self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M','J','P','G'))
                self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.capture_width)
                self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.capture_height)
                self.cap.set(cv2.CAP_PROP_FPS, self.framerate)
            
            # 테스트 프레임 읽기
            ret, _ = self.cap.read()
            if not ret:
                raise RuntimeError("카메라에서 프레임을 읽을 수 없습니다")
            
            self.br = CvBridge()
            self.timer = self.create_timer(1.0/self.framerate, self.timer_callback)
            self.get_logger().info(f'USB 카메라 노드가 초기화되었습니다 (device_id: {self.device_id})')
            
        except Exception as e:
            self.get_logger().error(f"카메라 초기화 중 오류 발생: {str(e)}")
            if hasattr(self, 'cap'):
                self.cap.release()
            raise


    def timer_callback(self):
        retry_count = 0
        max_retries = 3
        
        while retry_count < max_retries:
            ret, frame = self.cap.read()
            if ret:
                self.publisher_.publish(self.br.cv2_to_imgmsg(frame, "bgr8"))
                return
            else:
                retry_count += 1
                self.get_logger().warning(f"프레임 캡처 실패 {retry_count}/{max_retries}")
    
        # 재연결 시도
        self.get_logger().warn("카메라 재연결 시도...")
        self.cap.release()
        self._initialize_camera()        
        
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
