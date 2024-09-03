import os
import json
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    # 카메라 감지 스크립트 실행
    detect_script = os.path.join(os.path.dirname(__file__), 'detect_cameras.py')
    detect_process = ExecuteProcess(
        cmd=['python3', detect_script],
        output='screen'
    )
    detect_output = detect_process.execute()
    
    # 감지된 카메라 목록 파싱
    camera_ids = json.loads(detect_output.strip())
    
    # 각 감지된 카메라에 대해 노드 생성
    nodes = []
    for i in camera_ids:
        nodes.append(Node(
            package='project_eye_of_odin',
            executable='csi_camera_node',
            name=f'csi_camera_node_{i}',
            parameters=[{'sensor_id': i}],
        ))
    
    return LaunchDescription(nodes)