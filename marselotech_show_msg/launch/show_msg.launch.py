from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='marselotech_show_msg',
            executable='marselotech_show_msg',
            output='screen'),
    ])