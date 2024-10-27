#!/usr/bin/env python3

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
    
def generate_launch_description():
    
    pkg = get_package_share_directory('otto_simulation')
    rviz_path = os.path.join(pkg,'rviz','display.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', rviz_path],
        output='screen')
        
    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, "launch", "otto_description.launch.py")
        )
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("gazebo_ros"),
                    "launch",
                    "gazebo.launch.py"
                )
            ]
        )
    )
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic", "robot_description",
            "-entity", "otto"
        ],
        output = "screen"
    )


    launch_description = LaunchDescription()
    
    launch_description.add_action(rviz)
    launch_description.add_action(robot)
    # launch_description.add_action(joint_state_publisher)
    launch_description.add_action(gazebo)
    launch_description.add_action(spawn_entity)
    return launch_description