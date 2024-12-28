import os
import json
import subprocess
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    detect_script = os.path.join(os.path.dirname(__file__), 'detect_cameras.py')
    json_output = os.path.join(os.path.dirname(__file__), 'detected_cameras.json')
    
    try:
        if not os.path.exists(json_output):
            print("카메라 감지 실행...")
            subprocess.run(['python3', detect_script], check=True)
        
        with open(json_output, 'r') as f:
            camera_data = json.load(f)

    except Exception as e:
        print(f"카메라 감지 오류: {e}")
        return LaunchDescription([])
    

    # unique namespace jetson_00x Numerical digit: Nano(ex. jetson_001), alphabet: AGX(ex. jetson_00A)
    nodes = []
    for i in camera_data.get('csi', []):
        nodes.append(Node(
            package='csi_camera',
            executable='csi_camera_node',
            namespace='jetson_001/cameras',
            name=f'csi_camera_node_{i}',
            parameters=[{'sensor_id': i}]
        ))

    for i in camera_data.get('usb', []):
        nodes.append(Node(
            package='csi_camera',
            executable='usb_camera_node',
            namespace='jetson_001/cameras',     # e.g. topic example: /jetson_001/cameras/usb_0/image_raw
            name=f'usb_camera_node_{i}',
            parameters=[{'device_id': i}]
        ))

    return LaunchDescription(nodes)
