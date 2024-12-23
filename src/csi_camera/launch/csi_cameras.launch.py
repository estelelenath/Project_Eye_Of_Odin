import os
import json
import subprocess
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    detect_script = os.path.join(os.path.dirname(__file__), 'detect_all_cameras.py')
    json_output = os.path.join(os.path.dirname(__file__), 'detected_cameras.json')
    
    try:
        subprocess.run(['python3', detect_script], check=True)
        with open(json_output, 'r') as f:
            camera_data = json.load(f)
    except Exception as e:
        print(f"카메라 감지 오류: {e}")
        return LaunchDescription([])
    
    nodes = []
    for i in camera_data.get('csi', []):
        nodes.append(Node(
            package='csi_camera',
            executable='csi_camera_node',
            namespace='jetson_001/cameras/csi',
            name=f'csi_camera_node_{i}',
            parameters=[{'sensor_id': i}]
        ))

    for i in camera_data.get('usb', []):
        nodes.append(Node(
            package='csi_camera',
            executable='usb_camera_node',
            namespace='jetson_001/cameras/usb',
            name=f'usb_camera_node_{i}',
            parameters=[{'device_id': i}]
        ))

    return LaunchDescription(nodes)
