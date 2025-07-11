from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ss_nav',
            executable='mission.py',
            namespace='drone1',
            output='screen',
            parameters=[
                {'x_goal': 10.0},
                {'y_goal': -2.0},
                {'z_goal': 0.5},
                {'yaw_goal': 1.57}
            ]
        )
    ])