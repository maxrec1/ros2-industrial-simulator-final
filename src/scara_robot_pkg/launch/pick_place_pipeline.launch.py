#!/usr/bin/python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    scara_pkg = get_package_share_directory('scara_robot_pkg')
    moveit_pkg = get_package_share_directory('scara_moveit_config')

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(scara_pkg, 'launch', 'scara_conveyor_gazebo.launch.py')
        ),
        launch_arguments={'launch_rviz': 'False'}.items(),
    )

    moveit_launch = TimerAction(
        period=6.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(moveit_pkg, 'launch', 'demo_with_controllers.launch.py')
                ),
                launch_arguments={
                    'use_gazebo_controllers': 'True',
                    'rviz_tutorial': 'False',
                    'db': 'False',
                }.items(),
            )
        ],
    )

    scene_publisher = TimerAction(
        period=9.0,
        actions=[
            Node(
                package='scara_robot_pkg',
                executable='scene_publisher',
                output='screen',
                parameters=[
                    {
                        'planning_frame': 'world',
                        'dynamic_model_names': ['pcb1', 'chip1'],
                        'dynamic_update_hz': 5.0,
                    }
                ],
            )
        ],
    )

    pick_place_config = os.path.join(scara_pkg, 'config', 'pick_place_joints.yaml')
    pick_place_cycle = TimerAction(
        period=14.0,
        actions=[
            Node(
                package='scara_robot_pkg',
                executable='pick_place_cycle',
                output='screen',
                parameters=[{'config_path': pick_place_config}],
            )
        ],
    )

    # Spawn PCB on conveyor1 (belt1) with delay
    spawn_pcb = TimerAction(
        period=12.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'run', 'ros2_conveyorbelt', 'SpawnObject.py',
                     '--package', 'conveyorbelt_gazebo',
                     '--urdf', 'pcb.urdf',
                     '--name', 'pcb1',
                     '--x', '0.0',
                     '--y', '-0.5',
                     '--z', '1.2'],
                output='screen'
            )
        ],
    )

    # Spawn CHIP on conveyor2 (belt2) with delay
    spawn_chip = TimerAction(
        period=13.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'run', 'ros2_conveyorbelt', 'SpawnObject.py',
                     '--package', 'conveyorbelt_gazebo',
                     '--urdf', 'chip.urdf',
                     '--name', 'chip1',
                     '--x', '-1.3',
                     '--y', '-1.0',
                     '--z', '1.2'],
                output='screen'
            )
        ],
    )

    return LaunchDescription([
        gazebo_launch,
        moveit_launch,
        scene_publisher,
        pick_place_cycle,
        spawn_pcb,
        spawn_chip,
    ])
