#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    IncludeLaunchDescription,
    RegisterEventHandler,
    DeclareLaunchArgument,
    SetEnvironmentVariable
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Set GAZEBO_MODEL_PATH dynamically
    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=os.path.join(
            get_package_share_directory('limo_description'),
            'models'
        )
    )

    # Declare world argument
    world = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(
            get_package_share_directory('limo_description'),
            'worlds',
            'ramp-test.world'
        )
    )

    # Paths and package directories
    sim_pkg = get_package_share_directory('otto_simulation')
    rviz_path = os.path.join(sim_pkg, 'rviz', 'display.rviz')

    # Nodes and launch files
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', rviz_path],
        output='screen'
    )

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
        launch_arguments={'world': LaunchConfiguration('world')}.items()
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic", "robot_description",
            "-entity", "otto",
            "-x", "0.0",
            "-y", "0.0",
            "-z", "1.0",
            '-Y', "0.0"
        ],
        output="screen"
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
        output='screen'
    )

    joy = Node(
        package='otto_controller',
        executable='otto_xbox.py',
        name='otto_xbox',
        output='screen'
    )

    # Launch description
    launch_description = LaunchDescription()

    # Add environment variable
    launch_description.add_action(set_gazebo_model_path)

    # Add world declaration
    launch_description.add_action(world)

    # Event Handlers for controllers
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

    # launch_description.add_action(
    #     RegisterEventHandler(
    #         event_handler=OnProcessExit(
    #             target_action=effort_controller_spawner,
    #             on_exit=[position_controller_spawner],
    #         )
    #     )
    # )

    # Add nodes and launch files
    launch_description.add_action(rviz)
    launch_description.add_action(robot)
    launch_description.add_action(gazebo)
    launch_description.add_action(spawn_entity)
    # launch_description.add_action(lqr_controller)

    launch_description.add_action(joy)

    return launch_description
