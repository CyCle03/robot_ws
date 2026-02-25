import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    my_pkg_share = get_package_share_directory('my_mapper_turtlebot_pkg')
    nav2_share = get_package_share_directory('nav2_bringup')
    default_nav2_params = os.path.join(my_pkg_share, 'config', 'nav2_params_stress_maze.yaml')
    default_map_yaml = os.path.join(my_pkg_share, 'maps', 'stress_maze_map.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml = LaunchConfiguration('map')
    nav2_params = LaunchConfiguration('nav2_params')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time if true',
    )
    declare_map = DeclareLaunchArgument(
        'map',
        default_value=default_map_yaml,
        description='Full path to map yaml file for Nav2 map_server',
    )
    declare_nav2_params = DeclareLaunchArgument(
        'nav2_params',
        default_value=default_nav2_params,
        description='Full path to Nav2 parameter yaml file',
    )

    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(my_pkg_share, 'launch', 'stress_maze_world.launch.py'),
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_share, 'launch', 'bringup_launch.py'),
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
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
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription(
        [
            declare_use_sim_time,
            declare_map,
            declare_nav2_params,
            world_launch,
            nav2_launch,
            explorer_node,
        ]
    )
