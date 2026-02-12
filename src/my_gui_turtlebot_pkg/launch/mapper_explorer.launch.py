from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_gui_turtlebot_pkg',
            executable='mapper_explorer',
            name='mapper_explorer',
            output='screen',
        ),
    ])
