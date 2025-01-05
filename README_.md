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


Environment Variable Setting.
```bash
# Ubuntu Version
if [ "$(lsb_release -rs)" = "20.04" ]; then
    source /opt/ros/foxy/setup.bash
elif [ "$(lsb_release -rs)" = "22.04" ]; then
    source /opt/ros/humble/setup.bash
fi
```

*if first Usage of Lidar
ls -l /dev/ttyUSB*  # 라이다 장치가 인식되는지 확인

sudo usermod -a -G dialout $USER  # 현재 사용자에게 시리얼 포트 접근 권한 부여
sudo chmod 666 /dev/ttyUSB0       # 또는 라이다가 연결된 포트에 직접 권한 부여

```bash
$ colcon build --symlink-install
$ source install/setup.bash
$ ros2 launch project_eye_of_odin project_eye_of_odin.launch.py
```

Topic Monitoring
```bash
$ ros2 topic list
$ ros2 topic echo /scan
$ ros2 topic echo /cameraN/image_raw
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


** Update (2024.12.23)
 + Add USB Camera

    - Created a meta-package 'Project Eye of Odin' integrating CSI camera, USB camera, and LIDAR
    - Added USB camera support with GStreamer pipeline

    - project_eye_of_odin.launch.py: Simultaneously launches CSI camera, USB camera, and LIDAR
    - Automatic camera detection and configuration

+ minor update for enviroment setting

    Environment Variable Setting:
    ```bash
    # Ubuntu Version
    if [ "$(lsb_release -rs)" = "20.04" ]; then
        source /opt/ros/foxy/setup.bash
    elif [ "$(lsb_release -rs)" = "22.04" ]; then
        source /opt/ros/humble/setup.bash
    fi

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


        ***in RVIZ2***
        Global Option / Fixed Frame 에 laser 입력


        요약하자면..
        런치 파일을 통한 초기 설정 -> 노드 생성 및 파라미터 초기화 -> 하드웨어 연결 및 설정 -> 데이터 수집 및 처리 -> ROS2 메시지 발행
        1. generate_launch_description()를 통해 channel_type, serial_port, serial_baudrate 등 LiDAR 동작에 필요한 파라미터를 정의합니다.
        2. 이 파라미터들을 인자로 받아 sllidar_node를 실행하는 Node() 액션이 등록됩니다.
        3. launch 파일 실행 시, sllidar_node가 실행되며 지정된 시리얼 포트와 속도로 LiDAR 센서에 연결합니다.
        4. LiDAR 장치로부터 데이터가 들어오면 sllidar_node는 이를 해석하고, sensor_msgs::msg::LaserScan 형식으로 변환한 뒤 ROS2 토픽(scan)으로 퍼블리시합니다.
        5. 이 데이터는 이후 SLAM, Navigation, Visualization(RViz) 등 다른 ROS 노드에서 활용할 수 있습니다.



        ### Future Work.
        현재 Launchfile 에서 namespace가 하드 코딩되어있습니다.
        하지만 미래에는 자동화 되어야합니다.
        하지만 namespace같은 경우에 런치파일에서 node가 생성된이후에는 변경이 어렵습니다.
        따라서 sllidar_node 에서 퍼블리싱 되기전에 

    
        ```
            public:
                SLlidarNode()
                : Node("sllidar_node")
                {
                
                // this is our current method, create node 
                scan_pub = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", rclcpp::QoS(rclcpp::KeepLast(10)));
                
                // like below should change, just create default and later with serial name, unique topic create
                // scan_pub = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);
                
                }
        ```

        and also s2 or c1 lidar, we should integrate all... but we can hard coding...  

    

Process.
1. ROS2 (bridge_node) 에서 센서 데이터를 ZeroMQ로 퍼블리시(PUB).
2. observer/bridge (zmq_bridge) 에서 SUB로 수신 후, WebSocket을 통해 FastAPI 서버로 전달.
3. 브라우저는 FastAPI 서버에 접속(WebSocket), 카메라 영상·LiDAR 스캔을 실시간으로 수신·시각화.

(1) ROS2 토픽 → ZMQ로 전달
ROS2에서 카메라 토픽을 구독(sub)하는 노드를 만듭니다.
해당 노드가 ZMQ를 이용하여 다른 플랫폼(예: Python 서버, C++ 프로그램 등)으로 카메라 데이터를 발행(pub)합니다.
이렇게 하면 ROS2가 아닌 외부 프로그램도 ZMQ로부터 데이터를 받아볼 수 있습니다.
(2) ZMQ를 받은 외부 프로그램 → WebSocket → 브라우저
위에서 받은 카메라 데이터를 가공/인코딩 후, WebSocket 서버를 통해 브라우저로 전송합니다.
브라우저는 단순히 WebSocket을 열어서 실시간 영상을 수신하면 됩니다.

Requirement

sudo apt install uvicorn
sudo pip install fastapi uvicorn[standard] jinja2 aiofiles
pip install pyzmq

Run


USB Connection Environment

# 1. Robot) 먼저 ROS2 시스템 실행
ros2 launch project_eye_of_odin project_eye_of_odin.launch.py

# 2. Robot) odin_bridge 실행 (새 터미널)
ros2 launch odin_bridge odin_bridge.launch.py jetson_ids:='["001"]'

# 3. Robot) FastAPI 서버 실행 (새 터미널)
cd src/observer
uvicorn app.main:app --reload --host 0.0.0.0 --port 8000

동일 머신(로컬) 테스트 시 "Jetson Side Check"
http:// 127.0.0.1:8000

원격 접속(임베디드 ↔ Host PC) 시 "Host PC Side Check"
http://<임베디드-IP>:8000
http://192.168.55.1:8000



For Wifi



For 5G
future solution: VPN(WireGuard/OpenVPN)