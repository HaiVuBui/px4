from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='px4_takeoff',
            executable='takeoff_node',
            name='takeoff_node',
            output='screen'
        )
    ])