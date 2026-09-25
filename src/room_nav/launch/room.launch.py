from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    config = os.path.join(get_package_share_directory('room_nav'), 'rviz', 'room.rviz')
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Open RViz; set false for a headless run.'),
        Node(package='room_nav', executable='simulator', output='screen'),
        Node(package='room_nav', executable='navigator', output='screen'),
        Node(package='rviz2', executable='rviz2', arguments=['-d', config],
             condition=IfCondition(LaunchConfiguration('rviz')), output='screen'),
    ])
