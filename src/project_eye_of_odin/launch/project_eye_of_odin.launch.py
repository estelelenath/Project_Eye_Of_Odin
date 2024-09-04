import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    csi_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('csi_camera'), 'launch', 'csi_cameras.launch.py')
        ])
    )

    sllidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('lidar'), 'launch', 'sllidar_s2_launch.py')
        ])
    )

    return LaunchDescription([
        csi_camera_launch,
        sllidar_launch
    ])
