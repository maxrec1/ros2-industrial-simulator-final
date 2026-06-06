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
    bobby_pkg = get_package_share_directory('bobby')

    # ── Process Bobby URDF → replace package:// URIs with file:// paths ───────
    bobby_urdf_path = os.path.join(bobby_pkg, 'urdf', 'bobby.urdf')
    with open(bobby_urdf_path, 'r', encoding='utf-8') as f:
        bobby_description_raw = f.read()
    # Strip XML declaration — lxml rejects Unicode strings with encoding declarations
    if bobby_description_raw.startswith('<?xml'):
        bobby_description_raw = bobby_description_raw[bobby_description_raw.index('?>') + 2:].lstrip()
    bobby_description_raw = bobby_description_raw.replace(
        'package://bobby/', f'file://{bobby_pkg}/'
    )
    # Rename Bobby links with bobby_ prefix to avoid TF conflicts with SCARA's base_link etc.
    for link in ['base_link', 'link_1', 'link_2', 'link_3', 'link_4',
                 'link_5', 'link_6', 'link_7', 'link_8', 'TCP']:
        bobby_description_raw = bobby_description_raw.replace(
            f'name="{link}"', f'name="bobby_{link}"'
        )
        bobby_description_raw = bobby_description_raw.replace(
            f'link="{link}"', f'link="bobby_{link}"'
        )
    # Make Bobby static in Gazebo so it doesn't fall under gravity
    bobby_description_raw = bobby_description_raw.replace(
        '</robot>',
        '  <gazebo><static>true</static></gazebo>\n</robot>'
    )
    

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

    # ── Bobby Joint State Publisher (zero positions for all Bobby joints) ─────
    bobby_joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='bobby_joint_state_publisher',
        parameters=[{'robot_description': bobby_description_raw}],
        remappings=[
            ('/robot_description', '/bobby_description'),
            ('/joint_states', '/bobby_joint_states'),
        ],
        output='screen',
    )

    # ── Bobby Robot State Publisher (on separate topic) ──────────────────────
    bobby_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='bobby_state_publisher',
        parameters=[{'robot_description': bobby_description_raw}],
        remappings=[
            ('/robot_description', '/bobby_description'),
            ('/joint_states', '/bobby_joint_states'),
        ],
        output='screen',
    )

    # ── Static TF: world → bobby_base_link (places Bobby in RViz at pedestal top) ──
    bobby_world_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='bobby_world_tf',
        arguments=['0', '1.5', '0.8', '0', '0', '0', 'world', 'bobby_base_link'],
        output='screen',
    )

    # ── Spawn Bobby pedestal ──────────────────────────────────────────────────
    spawn_bobby_pedestal = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-file', pedestal_urdf,
            '-entity', 'bobby_pedestal',
            '-x', '0.0',
            '-y', '1.5',
            '-z', '0.0',
        ],
        output='screen',
    )

    # ── Spawn Bobby robot (on top of pedestal at z=0.8) ───────────────────────
    spawn_bobby = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', '/bobby_description',
            '-entity', 'bobby',
            '-x', '0.0',
            '-y', '1.5',
            '-z', '0.8',
        ],
        output='screen',
    )

    return LaunchDescription([
        launch_rviz_arg,
        gazebo_model_path,
        gazebo,
        robot_state_publisher,
        bobby_joint_state_publisher,
        bobby_state_publisher,
        bobby_world_tf,
        spawn_pedestal,
        spawn_scara,
        spawn_bobby_pedestal,
        spawn_bobby,
        spawn_controllers,
        rviz_node,
    ])
