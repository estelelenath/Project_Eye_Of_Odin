import rclpy  # ROS2 Python 클라이언트 라이브러리
from rclpy.node import Node  # ROS2 노드 클래스
from sensor_msgs.msg import Image  # 이미지 메시지 타입
from cv_bridge import CvBridge  # OpenCV와 ROS 이미지 변환 도구
import cv2  # OpenCV 라이브러리

def gstreamer_pipeline(
    sensor_id=0,
    capture_width=1280,
    capture_height=720,
    framerate=30,
):
    # GStreamer 파이프라인 문자열을 생성합니다.
    return (
        f"nvarguscamerasrc sensor-id={sensor_id} ! "
        f"video/x-raw(memory:NVMM), width=(int){capture_width}, height=(int){capture_height}, "
        f"framerate=(fraction){framerate}/1 ! "
        f"nvvidconv ! video/x-raw, format=(string)BGRx ! "
        "videoconvert ! video/x-raw, format=(string)BGR ! appsink"
    )

class CSICameraNode(Node):
    # 이 클래스는 추후에 아래에서 노드 인스턴트생성이라며, 객체가 생성될때 초기화됩니다. 
    # (모든 객체는 생성될때 초기화 됨.) 여튼 이렇게 초기화될때 def __init__ 는 무조건 한번 호출되며, __init__() 메서드 안에 있는 모든 코드가 순차적으로 실행됩니다. 
    # 먼저 설명하자면, 여기서 주기적으로 타이머가 콜백되는데, 
    # 여기서 호출되는 타이머 콜백을 보면 self.timer = self.create_timer(1.0/self.framerate, self.timer_callback) 에서 self.publisher_.publish(self.br.cv2_to_imgmsg(frame, "bgr8")) 가 있음. 
    # self.br.cv2_to_imgmsg(frame, "bgr8") :  cv2_to_imgmsg() 함수는 OpenCV에서 캡처한 frame을 ROS2의 sensor_msgs/Image 형식으로 변환 
    # 그리고 self.publisher_.publish 가 publish 함. self.br = CvBridge()  # CV 브릿지 객체 생성 : 가 실행되어 CvBridge 객체가 먼저 생성됩니다. 
    # 그리고 이건 프래임 혹은 오픈cv 이미지를 로스 메세지로 바꿀수있게 해주는 도구같은거임. 
    # 여튼 그래서 오해했던게, self.br = CvBridge() 가 나중에 호출되고 self.create_timer 가 먼저 호출되길래, 
    # 도구가 먼저 호출되어야 사용할수있는거 아닌가, 했지만 __init__ 안에서는 처음 한번 호출될때 다 순차적으로 호출됨, 
    # 따라서 순서는 중요한게 아님.. 
    # 처음부터 앞에 호출하지... (리소스 초기화 → 타이머 설정 순서로 작성하는 것이 좋습니다. 라고 하긴함, 하긴 코드도 클라우드물어봤으니 뭐..)
    def __init__(self):
        super().__init__('csi_camera_node')  # ROS2 노드 초기화
        self.declare_parameter('sensor_id', 0)  # sensor_id 파라미터 선언
        
        self.sensor_id = self.get_parameter('sensor_id').value  # 파라미터 값 가져오기
        self.framerate = 30  # 고정 프레임레이트 설정
        # 이미지 발행을 위한 Publisher 생성
        self.publisher_ = self.create_publisher(Image, f'camera{self.sensor_id}/image_raw', 10)
        # 타이머 콜백 설정
        self.timer = self.create_timer(1.0/self.framerate, self.timer_callback)
        # 카메라 캡처 객체 생성
        self.cap = cv2.VideoCapture(gstreamer_pipeline(sensor_id=self.sensor_id, framerate=self.framerate), cv2.CAP_GSTREAMER)
        self.br = CvBridge()  # CV 브릿지 객체 생성
        self.get_logger().info(f'CSI Camera Node initialized with sensor_id: {self.sensor_id}')

    def timer_callback(self):
        ret, frame = self.cap.read()  # 카메라에서 프레임 읽기
        if ret:
            # 읽은 프레임을 ROS 이미지 메시지로 변환하여 발행
            self.publisher_.publish(self.br.cv2_to_imgmsg(frame, "bgr8"))
        else:
            self.get_logger().warning("Failed to capture image")

    # 이건 객체가 파괴되면 자동으로 del은 불려져서 카메라 리소스는 해제됨, 처음 초기화가 자동으로 되는거처럼, 그래서 따로 del은 불릴필요없음
    def __del__(self):
        self.cap.release()  # 카메라 리소스 해제

def main(args=None):
    rclpy.init(args=args)  # ROS2 초기화
    csi_camera_node = CSICameraNode()  # 노드 인스턴스 생성
    # rclpy.spin(csi_camera_node)  # 노드 실행, 
    # 즉 카메라가 여러개 있다면 여기서 1번 카메라 작동! 2번 카메라 작동시작 이런느낌으로 시작되는거, 
    # 그리고 지속적인 컨티뉴 컨티뉴 이느낌은 타이머 콜백이 하는거고...
    rclpy.spin(csi_camera_node)  # 노드 실행
    csi_camera_node.destroy_node()  # 노드 정리
    rclpy.shutdown()  # ROS2 종료

if __name__ == '__main__':
    main()
