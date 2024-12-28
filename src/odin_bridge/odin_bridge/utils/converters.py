#!/usr/bin/env python3
import cv2
import numpy as np
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge

class DataConverter:
    _cv_bridge = CvBridge()

    @staticmethod
    def image_to_bytes(ros_image: Image) -> bytes:
        """ROS Image 메시지를 JPEG 바이트로 변환"""
        try:
            # ROS 이미지를 OpenCV 형식으로 변환
            cv_image = DataConverter._cv_bridge.imgmsg_to_cv2(
                ros_image, 
                desired_encoding='bgr8'
            )
            
            # JPEG 압축
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 85]
            _, jpeg_data = cv2.imencode('.jpg', cv_image, encode_param)
            
            return jpeg_data.tobytes()

        except Exception as e:
            raise Exception(f"Image conversion error: {str(e)}")

    @staticmethod
    def lidar_to_json(scan: LaserScan) -> dict:
        """LiDAR LaserScan 메시지를 JSON 형식으로 변환"""
        try:
            scan_dict = {
                'header': {
                    'stamp': {
                        'sec': scan.header.stamp.sec,
                        'nanosec': scan.header.stamp.nanosec
                    },
                    'frame_id': scan.header.frame_id
                },
                'angle_min': float(scan.angle_min),
                'angle_max': float(scan.angle_max),
                'angle_increment': float(scan.angle_increment),
                'time_increment': float(scan.time_increment),
                'scan_time': float(scan.scan_time),
                'range_min': float(scan.range_min),
                'range_max': float(scan.range_max),
                'ranges': [float(r) if not np.isinf(r) else None for r in scan.ranges],
                'intensities': [float(i) for i in scan.intensities] if scan.intensities else []
            }
            return scan_dict
        except Exception as e:
            raise Exception(f"LiDAR data conversion error: {str(e)}")
        
        