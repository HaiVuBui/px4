from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    takeoff_height = DeclareLaunchArgument(
        'takeoff_height', default_value='5.0', description='Takeoff height in meters (up, will be sent as negative NED z)'
    )
    goal_x = DeclareLaunchArgument('goal_x', default_value='5.0', description='Goal X in meters (NED, north is +X)')
    goal_y = DeclareLaunchArgument('goal_y', default_value='0.0', description='Goal Y in meters (NED, east is +Y)')
    goal_z = DeclareLaunchArgument(
        'goal_z',
        default_value='-5.0',
        description='Goal Z in meters (NED, down is +Z so use negative to be above takeoff point)',
    )
    goal_yaw = DeclareLaunchArgument('goal_yaw', default_value='0.0', description='Goal yaw in radians')
    hold_time = DeclareLaunchArgument('hold_time', default_value='3.0', description='Time to stay at goal before hold')
    tol_xy = DeclareLaunchArgument('position_tolerance_xy', default_value='0.5', description='XY tolerance in meters')
    tol_z = DeclareLaunchArgument('position_tolerance_z', default_value='0.5', description='Z tolerance in meters')

    node = Node(
        package='px4_takeoff',
        executable='mission_node',
        name='mission_node',
        output='screen',
        parameters=[
            {
                'takeoff_height': LaunchConfiguration('takeoff_height'),
                'goal_x': LaunchConfiguration('goal_x'),
                'goal_y': LaunchConfiguration('goal_y'),
                'goal_z': LaunchConfiguration('goal_z'),
                'goal_yaw': LaunchConfiguration('goal_yaw'),
                'hold_time': LaunchConfiguration('hold_time'),
                'position_tolerance_xy': LaunchConfiguration('position_tolerance_xy'),
                'position_tolerance_z': LaunchConfiguration('position_tolerance_z'),
            }
        ],
    )

    return LaunchDescription(
        [takeoff_height, goal_x, goal_y, goal_z, goal_yaw, hold_time, tol_xy, tol_z, node]
    )
