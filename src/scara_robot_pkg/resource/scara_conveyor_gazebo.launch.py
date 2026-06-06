#!/usr/bin/python3
# scara_conveyor_gazebo.launch.py
# Spawns SCARA robot + conveyor belt together in Gazebo with full ros2_control

import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz',
        default_value='False',
        description='Launch RViz with the Gazebo stack',
    )
    launch_rviz = LaunchConfiguration('launch_rviz')

    # ── Package paths ──────────────────────────────────────────────────────────
    scara_pkg   = get_package_share_directory('scara_robot_pkg')
    belt_pkg    = get_package_share_directory('conveyorbelt_gazebo')

    # ── Process SCARA xacro → robot_description ───────────────────────────────
    scara_xacro_file = os.path.join(scara_pkg, 'urdf', 'scara_gazebo.urdf.xacro')
    robot_description_raw = xacro.process_file(scara_xacro_file).toxml()
    # Replace package:// URIs with absolute file:// paths so Gazebo can find the meshes
    robot_description_raw = robot_description_raw.replace(
        'package://scara_robot_pkg/', f'file://{scara_pkg}/'
    )
    robot_description = {'robot_description': robot_description_raw}

    # ── Extend GAZEBO_MODEL_PATH so Gazebo can find conveyor_belt_2 model ──────
    belt2_models_path = os.path.join(scara_pkg, 'models')
    belt1_models_path = os.path.join(
        get_package_share_directory('conveyorbelt_gazebo'), 'models'
    )
    gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=belt2_models_path + ':' + belt1_models_path + ':' +
              os.environ.get('GAZEBO_MODEL_PATH', ''),
    )

    # ── Gazebo with two-conveyor world ────────────────────────────────────────
    world_file = os.path.join(scara_pkg, 'worlds', 'two_conveyors.world')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_file}.items(),
    )

    # ── Robot State Publisher ──────────────────────────────────────────────────
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    # ── Spawn cylinder pedestal under the robot ───────────────────────────────
    pedestal_urdf = os.path.join(scara_pkg, 'urdf', 'pedestal.urdf')
    spawn_pedestal = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-file', pedestal_urdf,
            '-entity', 'scara_pedestal',
            '-x', '-1.0',
            '-y', '0.0',
            '-z', '0.0',
        ],
        output='screen',
    )

    # ── Spawn SCARA into Gazebo ────────────────────────────────────────────────
    spawn_scara = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'scara_robot',
        ],
        output='screen',
    )

    # ── ros2_controllers.yaml path ─────────────────────────────────────────────
    ros2_controllers_path = os.path.join(
        get_package_share_directory('scara_moveit_config'),
        'config',
        'ros2_controllers.yaml',
    )

    # ── Spawn controllers after SCARA is loaded ────────────────────────────────
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '-c', '/controller_manager'],
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_trajectory_controller', '-c', '/controller_manager'],
    )

    # ── RViz2 for visualization ───────────────────────────────────────────────
    rviz_config = os.path.join(
        get_package_share_directory('scara_moveit_config'), 'config', 'moveit.rviz'
    )
    rviz_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_config],
                parameters=[
                    {'robot_description': robot_description_raw},
                ],
                output='log',
            )
        ],
        condition=IfCondition(launch_rviz),
    )

    # Start controllers only after SCARA entity is spawned
    spawn_controllers = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_scara,
            on_exit=[
                joint_state_broadcaster_spawner,
                arm_controller_spawner,
            ],
        )
    )

    return LaunchDescription([
        launch_rviz_arg,
        gazebo_model_path,
        gazebo,
        robot_state_publisher,
        spawn_pedestal,
        spawn_scara,
        spawn_controllers,
        rviz_node,
    ])
