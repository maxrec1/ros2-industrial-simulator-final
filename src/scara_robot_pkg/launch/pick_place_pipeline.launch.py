#!/usr/bin/python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    scara_pkg = get_package_share_directory('scara_robot_pkg')
    # Ensure locally built packages (e.g. linkattacher_msgs) are importable
    _ws_root = os.path.join(os.path.dirname(__file__), '..', '..', '..', '..', '..')
    _ws_root = os.path.normpath(_ws_root)
    _local_pypath = os.path.join(_ws_root, 'install', 'linkattacher_msgs',
                                 'local', 'lib', 'python3.10', 'dist-packages')
    _local_libpath = os.path.join(_ws_root, 'install', 'linkattacher_msgs', 'lib')
    _gazebo_plugin_path = os.path.join(_ws_root, 'install', 'ros2_linkattacher', 'lib')
    pythonpath_setup = SetEnvironmentVariable(
        name='PYTHONPATH',
        value=_local_pypath + ':' + os.environ.get('PYTHONPATH', ''),
    )
    ldpath_setup = SetEnvironmentVariable(
        name='LD_LIBRARY_PATH',
        value=_local_libpath + ':' + _gazebo_plugin_path + ':' + os.environ.get('LD_LIBRARY_PATH', ''),
    )
    gazebo_plugin_path_setup = SetEnvironmentVariable(
        name='GAZEBO_PLUGIN_PATH',
        value=_gazebo_plugin_path + ':' + os.environ.get('GAZEBO_PLUGIN_PATH', ''),
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(scara_pkg, 'launch', 'scara_conveyor_gazebo.launch.py')
        ),
        launch_arguments={'launch_rviz': 'True'}.items(),
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

    sonar_belt_stopper = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='scara_robot_pkg',
                executable='sonar_belt_stopper',
                output='screen',
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
                # Temporary disable: keep sonar active but hide belt2 ready event from this node.
                #remappings=[('belt2/object_ready', 'belt2/object_ready_disabled')],
                parameters=[
                    {
                        'config_path': pick_place_config,
                        # The robot entity spawned by scara_conveyor_gazebo.launch.py.
                        'robot_model_name': 'combined_cell',
                    }
                ],
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
        pythonpath_setup,
        ldpath_setup,
        gazebo_plugin_path_setup,
        gazebo_launch,
        scene_publisher,
        sonar_belt_stopper,
        pick_place_cycle,
        spawn_pcb,
        spawn_chip,
    ])
