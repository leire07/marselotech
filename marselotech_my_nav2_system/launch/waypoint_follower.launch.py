from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='marselotech_my_nav2_system',
            executable='waypoint_follower',
            output='screen')
    ])