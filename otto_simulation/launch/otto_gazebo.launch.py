#!/usr/bin/env python3

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
    
def generate_launch_description():
    
    sim_pkg = get_package_share_directory('otto_simulation')
    rviz_path = os.path.join(sim_pkg,'rviz','display.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', rviz_path],
        output='screen')
        
    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_pkg, "launch", "otto_description.launch.py")
        )
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            )
        ),
        launch_arguments={
            'pause': 'true'
        }.items()
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic", "robot_description",
            "-entity", "otto",
            "-x", "0.0",
            "-y", "0.0",   
            "-z", "0.38" 
        ],
        output = "screen"
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    effort_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["effort_controller", "--controller-manager", "/controller_manager"],
    )

    position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["position_controller", "--controller-manager", "/controller_manager"],
    )
    
    lqr_controller = Node(
        package='otto_controller',
        executable='joint_state_controller.py',
        name='lqr_controller',
        output='screen')

    launch_description = LaunchDescription()

    launch_description.add_action(
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[joint_state_broadcaster_spawner],
            )
        )
    )

    launch_description.add_action(
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[effort_controller_spawner],
            )
        )
    )

    launch_description.add_action(
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=effort_controller_spawner,
                on_exit=[position_controller_spawner],
            )
        )
    )

    # Add the rest of the nodes and launch descriptions
    launch_description.add_action(rviz)
    launch_description.add_action(robot)
    launch_description.add_action(gazebo)
    # launch_description.add_action(lqr_controller)
    launch_description.add_action(spawn_entity)

    return launch_description