import os
import json
import subprocess
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    print("Generating launch description...")  # Debug output

    # 카메라 감지 스크립트 실행
    detect_script = os.path.join(os.path.dirname(__file__), 'detect_cameras.py')
    json_output = os.path.join(os.path.dirname(__file__), 'detected_cameras.json')
    print(f"Detect script path: {detect_script}")  # Debug output

    try:
        subprocess.run(['python3', detect_script], check=True)
        with open(json_output, 'r') as f:
            camera_ids = json.load(f)
        print(f"Detected cameras: {camera_ids}")  # Debug output
    except subprocess.CalledProcessError as e:
        print(f"Error executing detect script: {e}")
        return LaunchDescription([])
    except json.JSONDecodeError as e:
        print(f"Error parsing JSON file: {e}")
        return LaunchDescription([])
    except FileNotFoundError:
        print(f"JSON file not found: {json_output}")
        return LaunchDescription([])

    nodes = []
    for i in camera_ids:
        print(f"Creating node for camera {i}")  # Debug output
        nodes.append(Node(
            package='project_eye_of_odin',
            executable='csi_camera_node',
            name=f'csi_camera_node_{i}',
            parameters=[{'sensor_id': i}],
        ))

    return LaunchDescription(nodes)
    
