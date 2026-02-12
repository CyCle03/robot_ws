from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_gui_turtlebot_pkg',
            executable='cmd_vel_arbiter',
            name='cmd_vel_arbiter',
            output='screen',
        ),
        Node(
            package='my_gui_turtlebot_pkg',
            executable='patrol_action_server',
            name='patrol_action_server',
            output='screen',
        ),
        Node(
            package='my_gui_turtlebot_pkg',
            executable='detect_obstacle',
            name='detect_obstacle',
            output='screen',
        ),
        Node(
            package='my_gui_turtlebot_pkg',
            executable='turtlebot_move_con',
            name='turtlebot_move_con',
            output='screen',
        ),
    ])
