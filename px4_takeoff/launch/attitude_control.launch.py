from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="px4_takeoff",
                executable="attitude_control_node",
                name="attitude_control_node",
                output="screen",
            ),
        ]
    )
