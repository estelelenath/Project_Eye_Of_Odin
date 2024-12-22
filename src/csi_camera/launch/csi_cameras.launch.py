import os  # 파일 경로 조작을 위한 모듈
import json  # JSON 파일 파싱을 위한 모듈
import subprocess  # 외부 프로세스 실행을 위한 모듈
from launch import LaunchDescription  # ROS2 launch 시스템의 핵심 클래스
from launch_ros.actions import Node  # ROS2 노드를 생성하기 위한 클래스

# def get_jetson_serial():
#     try:
#         with open('/proc/device-tree/serial-number', 'r') as f:
#             serial = f.read().strip()
#             return serial
#     except FileNotFoundError:
#         return "unknown_serial"

# jetson_serial = get_jetson_serial()
# namespace=f'jetson_{jetson_serial}',

def generate_launch_description():
    print("Launch 설명 생성 중...")  # 디버그 출력

    # 카메라 감지 스크립트의 경로를 설정합니다.
    detect_script = os.path.join(os.path.dirname(__file__), 'detect_cameras.py')
    json_output = os.path.join(os.path.dirname(__file__), 'detected_cameras.json')
    print(f"감지 스크립트 경로: {detect_script}")  # 디버그 출력

    try:
        # 카메라 감지 스크립트를 실행합니다.
        subprocess.run(['python3', detect_script], check=True)
        # 생성된 JSON 파일을 읽어 카메라 ID 목록을 가져옵니다.
        with open(json_output, 'r') as f:
            camera_ids = json.load(f)
        print(f"감지된 카메라: {camera_ids}")  # 디버그 출력
    except subprocess.CalledProcessError as e:
        print(f"감지 스크립트 실행 오류: {e}")
        return LaunchDescription([])  # 오류 시 빈 LaunchDescription 반환
    except json.JSONDecodeError as e:
        print(f"JSON 파일 파싱 오류: {e}")
        return LaunchDescription([])  # 오류 시 빈 LaunchDescription 반환
    except FileNotFoundError:
        print(f"JSON 파일을 찾을 수 없음: {json_output}")
        return LaunchDescription([])  # 오류 시 빈 LaunchDescription 반환

    nodes = []
    for i in camera_ids:
        print(f"카메라 {i}에 대한 노드 생성")  # 디버그 출력
        # 각 감지된 카메라에 대해 ROS2 노드를 생성합니다.
        nodes.append(Node(
            package='csi_camera',  # 패키지 이름
            executable='csi_camera_node',   # 실행 파일 이름 
                                            #(this execute the "csi_camera_node" in setup.py, 
                                            # if we defined, in setup this, we can execute in this.)
            namespace='jetson_001/sciCamera',  # unique namespace jetson_00x Numerical digit: Nano, alphabet: AGX
            name=f'csi_camera_node_{i}',    # 노드 이름
            parameters=[{'sensor_id': i}],  # 노드 파라미터
        ))

    return LaunchDescription(nodes)  # 생성된 노드 목록으로 LaunchDescription 반환, 
    # 이건 리스트의 값일뿐, 
    # 하지만 LaunchDescription(nodes)가 반환되면 ROS2 런치 시스템은 이를 받아 각 노드를 **출판(publish)**하고, 노드가 서로 통신할 수 있는 상태가 됩니다.
