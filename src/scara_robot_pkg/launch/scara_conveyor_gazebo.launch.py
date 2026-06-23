#!/usr/bin/python3
# scara_conveyor_gazebo.launch.py
# Combined cell: SCARA + bobby (movable, MoveIt-enabled) + conveyors in Gazebo.
#
# Both robots live in ONE robot description (combined_gazebo.urdf, produced by
# scripts/gen_combined_gazebo.py) driven by ONE global gazebo_ros2_control
# plugin / controller_manager. This is deliberate: gazebo_ros2_control 0.4.10
# writes a plugin's <ros><namespace> into the PROCESS-GLOBAL rcl arguments, so
# two namespaced plugins in one gzserver clobber each other and scatter their
# controllers across namespaces. A single global controller_manager with
# uniquely named controllers avoids the bug.

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
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
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz',
        default_value='False',
        description='Launch bobby MoveIt RViz with the Gazebo stack',
    )
    launch_rviz = LaunchConfiguration('launch_rviz')

    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='True',
        description='Start the Gazebo client GUI (set False for headless runs)',
    )
    gui = LaunchConfiguration('gui')

    # ── Package paths ──────────────────────────────────────────────────────────
    scara_pkg = get_package_share_directory('scara_robot_pkg')
    bobby_pkg = get_package_share_directory('bobby')
    bobby_moveit_share = get_package_share_directory('bobby_moveit_config_gazebo')

    # Locally built link-attacher packages are not always on Gazebo's plugin path
    # when this launch is started directly.
    ws_root = os.path.normpath(os.path.join(scara_pkg, '..', '..', '..', '..'))
    linkattacher_pythonpath = SetEnvironmentVariable(
        name='PYTHONPATH',
        value=os.path.join(ws_root, 'install', 'linkattacher_msgs', 'local', 'lib',
                           'python3.10', 'dist-packages') + ':' + os.environ.get('PYTHONPATH', ''),
    )
    linkattacher_ldpath = SetEnvironmentVariable(
        name='LD_LIBRARY_PATH',
        value=os.path.join(ws_root, 'install', 'linkattacher_msgs', 'lib') + ':' +
              os.path.join(ws_root, 'install', 'ros2_linkattacher', 'lib') + ':' +
              os.environ.get('LD_LIBRARY_PATH', ''),
    )
    linkattacher_plugin_path = SetEnvironmentVariable(
        name='GAZEBO_PLUGIN_PATH',
        value=os.path.join(ws_root, 'install', 'ros2_linkattacher', 'lib') + ':' +
              os.environ.get('GAZEBO_PLUGIN_PATH', ''),
    )

    # ── Combined robot_description (SCARA + bobby in one model) ────────────────
    combined_urdf = os.path.join(scara_pkg, 'urdf', 'combined_gazebo.urdf')
    combined_controllers_yaml = os.path.join(scara_pkg, 'config', 'combined_controllers.yaml')
    with open(combined_urdf, 'r', encoding='utf-8') as f:
        robot_description_raw = f.read()
    if robot_description_raw.startswith('<?xml'):
        robot_description_raw = robot_description_raw[robot_description_raw.index('?>') + 2:].lstrip()
    # Gazebo needs absolute file:// mesh URIs; both packages contribute meshes.
    robot_description_raw = robot_description_raw.replace(
        'package://scara_robot_pkg/', f'file://{scara_pkg}/')
    robot_description_raw = robot_description_raw.replace(
        'package://bobby/', f'file://{bobby_pkg}/')
    # Point the single gazebo plugin at the combined controllers.yaml.
    robot_description_raw = robot_description_raw.replace(
        '__COMBINED_CONTROLLERS_YAML__', combined_controllers_yaml)
    robot_description = {'robot_description': robot_description_raw}

    # ── MoveIt config for the combined cell ──────────────────────────────────
    # One move_group loads both robots as a single kinematic model. The SRDF
    # defines scara_arm, bobby_arm, bobby_gripper, and both_arms; the controller
    # config keeps execution split across the two arm trajectory controllers.
    combined_srdf = os.path.join(scara_pkg, 'config', 'combined.srdf')
    combined_moveit = (
        MoveItConfigsBuilder('combined_cell', package_name='scara_robot_pkg')
        .robot_description(file_path=combined_urdf)
        .robot_description_semantic(file_path=combined_srdf)
        .robot_description_kinematics(file_path=os.path.join(scara_pkg, 'config', 'kinematics.yaml'))
        .joint_limits(file_path=os.path.join(scara_pkg, 'config', 'joint_limits.yaml'))
        .trajectory_execution(file_path=os.path.join(scara_pkg, 'config', 'moveit_controllers.yaml'))
        .planning_pipelines(pipelines=['ompl'])
        .to_moveit_configs()
    )

    # ── Gazebo with the two-conveyor world ────────────────────────────────────
    belt2_models_path = os.path.join(scara_pkg, 'models')
    belt1_models_path = os.path.join(
        get_package_share_directory('conveyorbelt_gazebo'), 'models')
    gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=belt2_models_path + ':' + belt1_models_path + ':' +
              os.environ.get('GAZEBO_MODEL_PATH', ''),
    )
    world_file = os.path.join(scara_pkg, 'worlds', 'two_conveyors.world')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_file, 'gui': gui}.items(),
    )

    # ── Robot State Publisher (global, one for the whole cell) ────────────────
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}],
    )

    # ── Static TFs for sonar sensor frames ────────────────────────────────────
    belt1_sonar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='belt1_sonar_tf',
        arguments=['0', '0.6', '0.791', '0', '1.5708', '0', 'world', 'belt1_sonar_link'],
        output='screen',
    )
    belt2_sonar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='belt2_sonar_tf',
        arguments=['-0.70', '-1.0', '0.791', '-1.5708', '1.5708', '0', 'world', 'belt2_sonar_link'],
        output='screen',
    )

    # ── Pedestals under each robot ────────────────────────────────────────────
    pedestal_urdf = os.path.join(scara_pkg, 'urdf', 'pedestal.urdf')
    # -timeout 120: gzserver can take well over the spawn_entity default of 30 s to
    # bring up GazeboRosFactory (/spawn_entity) on slow/software-rendered machines;
    # a short timeout makes the spawns fail and the controller_manager never start.
    spawn_scara_pedestal = Node(
        package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-file', pedestal_urdf, '-entity', 'scara_pedestal',
                   '-x', '-1.0', '-y', '0.0', '-z', '0.0', '-timeout', '120.0'],
        output='screen',
    )
    spawn_bobby_pedestal = Node(
        package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-file', pedestal_urdf, '-entity', 'bobby_pedestal',
                   '-x', '0.0', '-y', '1.0', '-z', '0.0', '-timeout', '120.0'],
        output='screen',
    )
    # Drop box stands on its own pedestal (top at z=0.8), same x,y as the box.
    spawn_drop_box_pedestal = Node(
        package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-file', pedestal_urdf, '-entity', 'drop_box_pedestal',
                   '-x', '0.0', '-y', '1.45', '-z', '0.0', '-timeout', '120.0'],
        output='screen',
    )

    drop_box_urdf = os.path.join(scara_pkg, 'urdf', 'drop_box.urdf')
    spawn_drop_box = Node(
        package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-file', drop_box_urdf, '-entity', 'drop_box',
                   '-x', '0.0', '-y', '1.45', '-z', '0.7', '-timeout', '120.0'],
        output='screen',
    )

    # ── Spawn the combined robot model. Each robot is anchored by a fixed
    #    world joint in the URDF, so spawn at the origin. ──────────────────────
    spawn_combined = Node(
        package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'combined_cell',
                   '-timeout', '120.0'],
        output='screen',
    )

    # ── Controllers (all on the single global /controller_manager) ────────────
    joint_state_broadcaster_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster', '-c', '/controller_manager'],
    )
    scara_arm_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['arm_trajectory_controller', '-c', '/controller_manager'],
    )
    bobby_arm_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['bobby_arm_controller', '-c', '/controller_manager'],
    )
    bobby_gripper_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['bobby_gripper_controller', '-c', '/controller_manager'],
    )
    spawn_controllers = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_combined,
            on_exit=[
                joint_state_broadcaster_spawner,
                scara_arm_spawner,
                bobby_arm_spawner,
                bobby_gripper_spawner,
            ],
        )
    )

    # ── Combined move_group (MoveIt, global namespace) ────────────────────────
    combined_move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[combined_moveit.to_dict(), {'use_sim_time': True}],
    )

    # ── MoveIt planning scene: conveyors/pedestals as CollisionObjects ────────
    scene_publisher = TimerAction(
        period=9.0,
        actions=[Node(
            package='scara_robot_pkg',
            executable='scene_publisher',
            output='screen',
            parameters=[{
                'planning_frame': 'world',
                'dynamic_model_names': ['pcb1', 'chip1'],
                'dynamic_update_hz': 5.0,
            }],
        )],
    )

    sonar_belt_stopper = TimerAction(
        period=9.0,
        actions=[Node(
            package='scara_robot_pkg',
            executable='sonar_belt_stopper',
            output='screen',
        )],
    )

    # ── RViz with MotionPlanning (only with launch_rviz:=True) ────────────────
    combined_rviz = TimerAction(
        period=8.0,
        actions=[Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_combined_cell',
            arguments=['-d', os.path.join(bobby_moveit_share, 'config', 'moveit.rviz')],
            parameters=[
                combined_moveit.robot_description,
                combined_moveit.robot_description_semantic,
                combined_moveit.robot_description_kinematics,
                combined_moveit.planning_pipelines,
                combined_moveit.joint_limits,
                {'use_sim_time': True},
            ],
            output='log',
        )],
        condition=IfCondition(launch_rviz),
    )

    return LaunchDescription([
        launch_rviz_arg,
        gui_arg,
        linkattacher_pythonpath,
        linkattacher_ldpath,
        linkattacher_plugin_path,
        gazebo_model_path,
        gazebo,
        robot_state_publisher,
        belt1_sonar_tf,
        belt2_sonar_tf,
        spawn_scara_pedestal,
        spawn_bobby_pedestal,
        spawn_drop_box_pedestal,
        spawn_drop_box,
        spawn_combined,
        spawn_controllers,
        combined_move_group,
        scene_publisher,
        sonar_belt_stopper,
        combined_rviz,
    ])
