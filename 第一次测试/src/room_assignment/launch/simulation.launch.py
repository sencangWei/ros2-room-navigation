import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    gazebo_launch = os.path.join(get_package_share_directory('gazebo_ros'), 'launch')
    robot_launch = os.path.join(get_package_share_directory('wpr_simulation2'), 'launch')
    world = os.path.join(get_package_share_directory('room_assignment'), 'worlds', 'multiroom.world')
    gui = LaunchConfiguration('gui')
    model_path = os.path.dirname(get_package_share_directory('wpr_simulation2'))
    if os.environ.get('GAZEBO_MODEL_PATH'):
        model_path += os.pathsep + os.environ['GAZEBO_MODEL_PATH']

    return LaunchDescription([
        SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', ''),
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', model_path),
        DeclareLaunchArgument('gui', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gazebo_launch, 'gzserver.launch.py')),
            launch_arguments={'world': world}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gazebo_launch, 'gzclient.launch.py')),
            condition=IfCondition(gui)),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(robot_launch, 'spawn_wpb_lidar.launch.py')),
            launch_arguments={'pose_x': '-4.8', 'pose_y': '0.0', 'pose_theta': '0.0'}.items()),
    ])
