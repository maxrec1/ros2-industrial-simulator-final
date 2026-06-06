import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    tutorial_arg = DeclareLaunchArgument(
        "rviz_tutorial", default_value="False", description="Tutorial flag"
    )

    db_arg = DeclareLaunchArgument(
        "db", default_value="False", description="Database flag"
    )

    use_gazebo_controllers_arg = DeclareLaunchArgument(
        "use_gazebo_controllers",
        default_value="True",
        description="When true, do not start local ros2_control, static TF, or robot_state_publisher",
    )

    moveit_config = (
        MoveItConfigsBuilder("Scara Assembly1", package_name="scara_moveit_config")
        .robot_description_semantic(file_path="config/Scara Assembly1.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(
            pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"]
        )
        .to_moveit_configs()
    )


    # Start the actual move_group node/action server
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), {"use_sim_time": True}],
        arguments=["--ros-args", "--log-level", "info"],
    )

    # RViz
    tutorial_mode = LaunchConfiguration("rviz_tutorial")
    use_gazebo_controllers = LaunchConfiguration("use_gazebo_controllers")
    rviz_base = os.path.join(
        get_package_share_directory("scara_moveit_config"), "config"
    )
    rviz_full_config = os.path.join(rviz_base, "moveit.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_full_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            {"use_sim_time": True},
        ],
        condition=UnlessCondition(tutorial_mode),
    )
    rviz_node_tutorial = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_full_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            {"use_sim_time": True},
        ],
        condition=IfCondition(tutorial_mode),
    )

    # Static TF
    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["-1.0", "0.0", "0.8", "0.0", "0.0", "0.0", "world", "base_link"],
        parameters=[{"use_sim_time": True}],
        condition=UnlessCondition(use_gazebo_controllers),
    )

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description, {"use_sim_time": True}],
        condition=UnlessCondition(use_gazebo_controllers),
    )

    # ros2_control node
    ros2_controllers_path = os.path.join(
        get_package_share_directory("scara_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ros2_controllers_path],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        output="screen",
        condition=UnlessCondition(use_gazebo_controllers),
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        condition=UnlessCondition(use_gazebo_controllers),
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_trajectory_controller", "-c", "/controller_manager"],
        condition=UnlessCondition(use_gazebo_controllers),
    )

    # Warehouse mongodb server
    db_config = LaunchConfiguration("db")
    mongodb_server_node = Node(
        package="warehouse_ros_mongo",
        executable="mongo_wrapper_ros.py",
        parameters=[
            {"warehouse_port": 33829},
            {"warehouse_host": "localhost"},
            {"warehouse_plugin": "warehouse_ros_mongo::MongoDatabaseConnection"},
        ],
        output="screen",
        condition=IfCondition(db_config),
    )

    return LaunchDescription(
        [
            tutorial_arg,
            db_arg,
            use_gazebo_controllers_arg,
            rviz_node,
            rviz_node_tutorial,
            static_tf_node,
            robot_state_publisher,
            move_group_node,
            ros2_control_node,
            joint_state_broadcaster_spawner,
            arm_controller_spawner,
            mongodb_server_node,
        ]
    )
