from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Launch Arguments
    jetson_ids_arg = DeclareLaunchArgument(
        'jetson_ids',
        default_value='["001", "00A"]',  # 기본값으로 여러 Jetson ID 설정
        description='List of Jetson device IDs to monitor (in JSON array format)'
    )

    # Bridge Node
    bridge_node = Node(
        package='odin_bridge',
        executable='bridge_node',
        name='odin_bridge',
        parameters=[{
            'jetson_ids': LaunchConfiguration('jetson_ids')
        }],
        output='screen',
        emulate_tty=True
    )

    return LaunchDescription([
        jetson_ids_arg,
        bridge_node
    ])