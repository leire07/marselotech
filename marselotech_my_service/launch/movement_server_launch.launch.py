from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='marselotech_my_service',
            executable='movement_server',
            output='screen'
        ),
    ])