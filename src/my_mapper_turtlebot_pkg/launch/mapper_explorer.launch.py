from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time if true',
        ),
        Node(
            package='my_mapper_turtlebot_pkg',
            executable='mapper_explorer',
            name='mapper_explorer',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])
