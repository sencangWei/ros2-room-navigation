import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('room_assignment')
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'simulation.launch.py')),
            launch_arguments={'gui': LaunchConfiguration('gui')}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('slam_toolbox'),
                                                     'launch', 'online_async_launch.py')),
            launch_arguments={'use_sim_time': 'true',
                              'slam_params_file': os.path.join(share, 'config', 'slam.yaml')}.items()),
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', os.path.join(share, 'rviz', 'mapping.rviz')],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
