from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg = get_package_share_directory('bobby')

    use_gui = DeclareLaunchArgument(
        'use_gui',
        default_value='true',
        description='Use joint_state_publisher_gui to move joints'
    )

    urdf_arg = DeclareLaunchArgument(
        'urdf',
        default_value='bobby.urdf',
        description='URDF file name inside the urdf/ directory'
    )

    urdf_file = PathJoinSubstitution([pkg, 'urdf', LaunchConfiguration('urdf')])

    robot_description = ParameterValue(Command(['cat ', urdf_file]), value_type=str)

    # Publishes TF from the URDF — starts first so /robot_description topic is available
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen'
    )

    # Delayed start: gives robot_state_publisher time to publish /robot_description
    # so joint_state_publisher_gui can load URDF and handle mimic joints correctly
    jsp_gui = TimerAction(
        period=1.5,
        actions=[Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            parameters=[{
                'rate': 30,
                'use_mimic_tags': True,
            }],
            emulate_tty=True
        )]
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', PathJoinSubstitution([pkg, 'rviz', 'default_view.rviz'])],
        output='screen'
    )

    return LaunchDescription([
        use_gui,
        urdf_arg,
        rsp,
        jsp_gui,
        rviz,
    ])
