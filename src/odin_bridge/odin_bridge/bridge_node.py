#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
import zmq
import json
from .utils.converters import DataConverter

class OdinBridgeNode(Node):
    def __init__(self):
        super().__init__('odin_bridge')
        
        # ZMQ Publisher 설정
        self._zmq_context = zmq.Context()
        self._zmq_socket = self._zmq_context.socket(zmq.PUB)
        self._zmq_socket.bind("tcp://*:5555")
        
        # Jetson IDs 파라미터 받기
        self.declare_parameter('jetson_ids', ['001'])
        self.jetson_ids = self.get_parameter('jetson_ids').value
        
        # Subscribers for all Jetsons
        self.camera_subs = []
        camera_types = ['usb', 'csi']
        camera_indices = range(4)
        
        # 모든 Jetson에 대해 구독 설정
        for jetson_id in self.jetson_ids:
            # 카메라 토픽 구독
            for cam_type in camera_types:
                for idx in camera_indices:
                    topic = f'/jetson_{jetson_id}/cameras/{cam_type}_{idx}/image_raw'
                    try:
                        sub = self.create_subscription(
                            Image,
                            topic,
                            lambda msg, t=topic: self.camera_callback(msg, t),
                            10
                        )
                        self.camera_subs.append(sub)
                        self.get_logger().info(f'Subscribed to camera topic: {topic}')
                    except Exception as e:
                        self.get_logger().warn(f'Failed to subscribe to {topic}: {str(e)}')
            
            # LiDAR 토픽 구독
            lidar_topic = f'/jetson_{jetson_id}/scan'
            sub = self.create_subscription(
                LaserScan,
                lidar_topic,
                self.lidar_callback,
                10
            )
            self.get_logger().info(f'Subscribed to LiDAR topic: {lidar_topic}')

    def __del__(self):  # 이 메서드 추가
        if hasattr(self, '_zmq_socket'):
            self._zmq_socket.close()
        if hasattr(self, '_zmq_context'):
            self._zmq_context.destroy()

    def camera_callback(self, msg, topic_name):
        """카메라 이미지 처리 및 전송"""
        try:
            # 이미지 데이터 변환
            image_data = DataConverter.image_to_bytes(msg)
            
            # 메타데이터 준비
            metadata = {
                'topic': topic_name,
                'timestamp': {
                    'sec': msg.header.stamp.sec,
                    'nanosec': msg.header.stamp.nanosec
                },
                'frame_id': msg.header.frame_id,
                'height': msg.height,
                'width': msg.width
            }
            
            # 토픽과 데이터 전송
            self._zmq_socket.send_multipart([
                b"camera",
                json.dumps(metadata).encode(),
                image_data
            ])
            
            self.get_logger().debug(f'Sent image from topic: {topic_name}')
        except Exception as e:
            self.get_logger().error(f'Camera callback error: {str(e)}')

    def lidar_callback(self, msg):
        """LiDAR 데이터 처리 및 전송"""
        try:
            # LiDAR 데이터 변환
            lidar_data = DataConverter.lidar_to_json(msg)
            
            # 토픽과 데이터 전송
            self._zmq_socket.send_multipart([
                b"lidar",
                json.dumps(lidar_data).encode()
            ])
            
            self.get_logger().debug('Sent LiDAR data')
        except Exception as e:
            self.get_logger().error(f'Lidar callback error: {str(e)}')



def main(args=None):
    rclpy.init(args=args)
    node = OdinBridgeNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
