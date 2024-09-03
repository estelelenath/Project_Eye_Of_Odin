import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import numpy as np
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst

class GstreamerCameraNode(Node):
    def __init__(self):
        super().__init__('gstreamer_camera_node')
        self.publisher_ = self.create_publisher(Image, 'camera/image_raw', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10Hz
        
        Gst.init(None)
        self.pipeline = Gst.parse_launch(
            "nvarguscamerasrc ! video/x-raw(memory:NVMM),width=1920,height=1080,framerate=30/1,format=NV12 ! "
            "nvvidconv ! video/x-raw,format=BGRx ! videoconvert ! video/x-raw,format=BGR ! appsink"
        )
        self.pipeline.set_state(Gst.State.PLAYING)
        self.appsink = self.pipeline.get_by_name('appsink0')

    def timer_callback(self):
        sample = self.appsink.emit('pull-sample')
        if sample:
            buf = sample.get_buffer()
            caps = sample.get_caps()
            width = caps.get_structure(0).get_value('width')
            height = caps.get_structure(0).get_value('height')
            
            arr = np.ndarray(
                (height, width, 3),
                buffer=buf.extract_dup(0, buf.get_size()),
                dtype=np.uint8
            )
            
            img_msg = Image()
            img_msg.header.stamp = self.get_clock().now().to_msg()
            img_msg.header.frame_id = "camera_frame"
            img_msg.height = height
            img_msg.width = width
            img_msg.encoding = "bgr8"
            img_msg.is_bigendian = False
            img_msg.step = width * 3
            img_msg.data = arr.tobytes()
            
            self.publisher_.publish(img_msg)

def main(args=None):
    rclpy.init(args=args)
    node = GstreamerCameraNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
