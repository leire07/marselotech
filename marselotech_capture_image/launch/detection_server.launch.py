from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='marselotech_capture_image',
            executable='detection_server',
            output='screen'
        ),
    ])