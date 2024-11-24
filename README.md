# Project_Eye_Of_Odin
Comprehensive Information Integrated Processing System

1. Project Structure Modification:
    - Created a meta-package 'Project Eye of Odin' integrating CSI camera and LIDAR

2. New Launch File:
    - project_eye_of_odin.launch.py: Simultaneously launches CSI camera and LIDAR

3. Package Composition:
    - csi_camera: Controls CSI camera and publishes images
    - lidar: Controls SLLIDAR S2 and publishes scan data
    - project_eye_of_odin: Meta-package integrating both sensors


```bash
$ colcon build --symlink-install
$ source install/setup.bash
$ ros2 launch project_eye_of_odin project_eye_of_odin.launch.py
```

### csi_camera
* Process
    1. main Launch File execute
        
        $ ros2 launch project_eye_of_odin project_eye_of_odin.launch.py

        ```bash
        def generate_launch_description():
        csi_camera_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('csi_camera'), 'launch', 'csi_cameras.launch.py')
            ])
        )
        ```
        -> csi_camera package
    2. csi_cameras.launch.py

사실 런치 csi_cameras.launch 하면, camera detect되서 json 파일로 리스트 업되고, 그걸로 노드정보를 launchDsecription한다는 개념
그 사이에 카메라 영상을 불러오는 csi_camera_node.py이 호출되는 것뿐..
그리고 여기에 실질적으로 카메라를 불러오는 csi_camera_node를 포함하는 main함수는 setup에 포함되어 있기 때문에 우리가 사용할수있습니다.

그리고 런치파일이 런치 디스크립션을 리턴하는데, 거기에는 우리가 실행할 액션(노드나 이벤트 핸들러등등) 목록이 있습니다.


ROS2 런치시스템은 런치디스크립션의 노드를 실행합니다
이때에 각 노드들은 예를들어 여기서는 csi_camera_node 실행파일을 실행하며, 이는 앞서 말한바와같이 이미 Setup.py 에 정의되어있으며, 노드가 생성되어서 활성화(.spin) 되면 csi_camera_node 에서 이미 구현된대로 이미지가 캡쳐되고 ros가 인식할수있게 토픽으로 퍼블리시됩니다csi_camera_node를 포함하는 main함수는 setup에 포함되어 있기 때문에 우리가 사용할수있습니다.

그리고 런치파일이 런치 디스크립션을 리턴하는데, 거기에는 우리가 실행할 액션(노드나 이벤트 핸들러등등) 목록이 있습니다.


ROS2 런치시스템은 런치디스크립션의 노드를 실행합니다
이때에 각 노드들은 예를들어 여기서는 csi_camera_node 실행파일을 실행하며, 이는 앞서 말한바와같이 이미 Setup.py 에 정의되어있으며, 노드가 생성되어서 활성화(.spin) 되면 csi_camera_node 에서 이미 구현된대로 이미지가 캡쳐되고 ros가 인식할수있게 토픽으로 퍼블리시됩니다


Diagram

사용자 명령

$ ros2 launch project_eye_of_odin project_eye_of_odin.launch.py

▼

project_eye_of_odin.launch.py 실행

IncludeLaunchDescription로 하위 런치 파일 포함:

- csi_camera/launch/csi_cameras.launch.py
- sllidar_ros2/launch/sllidar_s2_launch.py    

▼

csi_cameras.launch.py 실행 / sllidar_s2_launch.py 실행

1. detect_cameras.py 실행
2. 카메라 감지 및 JSON 저장
3. 감지된 카메라 ID로 노드 생성
- 각 노드는 sensor_id 설정

/

(라이다 노드 실행)

▼

LaunchDescription 반환
- csi_camera_node 노드 목록 포함  

▼

ROS2 런치 시스템에 의해 노드 실행
- csi_camera_node 실행 파일 실행
(setup.py의 entry_points에서 정의됨)

▼


각 csi_camera_node.py의 main 함수 실행
- CSICameraNode 클래스 인스턴스 생성
- rclpy.spin으로 노드 활성화

▼

CSICameraNode 내에서 이미지 캡처 및 퍼블리시

- GStreamer 파이프라인으로 카메라 초기화
- 타이머 콜백으로 주기적으로 이미지 캡처
- CvBridge로 이미지를 ROS 메시지로 변환
- sensor_msgs/Image 토픽으로 퍼블리시


### Lidar

* Process
    1. main Launch File execute
        
        $ ros2 launch project_eye_of_odin project_eye_of_odin.launch.py

        ```bash
        csi_camera_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('csi_camera'), 'launch', 'csi_cameras.launch.py')
            ])
        )

        lidar_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('lidar'), 'launch', 'sllidar_s2_launch.py')
            ])
        )
        ```
        -> Lidar package
    2. Run **sllidar_s2_launch.py**
    
         - Setting for LIDAR Node to run
        here, sllidar_node is executed as action.
        

        ▼

         - ROS2 런치 시스템에 의해 sllidar_node 실행
         - sllidar_node 실행 파일 실행 (C++ 노드)
             - LIDAR 센서 초기화 (/dev/ttyUSB0 포트 사용)
            - 스캔 데이터 수집
            - **sensor_msgs/LaserScan** 토픽으로 데이터 퍼블리시

        * ** 
        sllidar_node.cpp: 이 파일은 LIDAR 센서의 드라이버 역할을 합니다.

        요약하자면..
        런치 파일을 통한 초기 설정 -> 노드 생성 및 파라미터 초기화 -> 하드웨어 연결 및 설정 -> 데이터 수집 및 처리 -> ROS2 메시지 발행


    



