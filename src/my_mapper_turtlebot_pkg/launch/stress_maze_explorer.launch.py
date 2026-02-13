import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    my_pkg_share = get_package_share_directory('my_mapper_turtlebot_pkg')
    nav2_share = get_package_share_directory('nav2_bringup')
    nav2_params = os.path.join(nav2_share, 'params', 'nav2_params.yaml')

    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(my_pkg_share, 'launch', 'stress_maze_world.launch.py'),
        ),
        launch_arguments={'use_sim_time': 'true'}.items(),
    )

    map_yaml = os.path.join(my_pkg_share, 'maps', 'stress_maze_map.yaml')
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_share, 'launch', 'bringup_launch.py'),
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'map': map_yaml,
            'params_file': nav2_params,
            'autostart': 'true',
            'use_composition': 'False',
            'use_respawn': 'False',
        }.items(),
    )

    explorer_node = Node(
        package='my_mapper_turtlebot_pkg',
        executable='mapper_explorer',
        name='mapper_explorer',
        output='screen',
    )

    return LaunchDescription(
        [
            world_launch,
            TimerAction(period=5.0, actions=[nav2_launch]),
            TimerAction(period=10.0, actions=[explorer_node]),
        ]
    )
