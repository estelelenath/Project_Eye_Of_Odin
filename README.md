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

$ colcon build --symlink-install
$ source install/setup.bash
$ ros2 launch project_eye_of_odin project_eye_of_odin.launch.py

