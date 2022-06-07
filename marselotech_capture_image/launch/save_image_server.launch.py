from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='marselotech_capture_image',
            executable='save_image_service',
            output='screen'
        ),
    ])