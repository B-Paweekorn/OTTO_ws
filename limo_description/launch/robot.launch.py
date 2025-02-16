from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Declare launch arguments
    world = DeclareLaunchArgument(
        'world', 
        default_value=PathJoinSubstitution([
            FindPackageShare('limo_description'), 'worlds', 'basic.world'
        ])
    )

    # Include another launch file
    gazebo_models_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('limo_description'), 'launch', 'gazebo_models_diff_mod_basics.launch.py'
            ])
        ),
        launch_arguments={
            'world': LaunchConfiguration('world')
        }.items()
    )



    # Return the LaunchDescription
    return LaunchDescription([
        world,
        gazebo_models_launch,
    ])
