import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    package_name = "robot_description"
    
    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare(package_name), "launch", "robot.launch.py"])]
        ),
        launch_arguments={"use_sim_time": "true"}.items(),
    )
    
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare("gazebo_ros"), "launch", "gazebo.launch.py"])]
        ),
        launch_arguments={"verbose": "false"}.items(),
    )
    
    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("robot_description"),
                    "urdf",
                    "robot_gazebo.urdf.xacro",
                ]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("robot_bringup"),
            "config",
            "robot_controllers.yaml",
        ]
    )
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("rrbot_description"), "config", "rrbot.rviz"]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
    )
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )
    
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "robot"],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_state"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["diff_drive"],
    )

    nodes = [
        control_node,
        # robot_state_pub_node,
        # rviz_node,
        robot,
        gazebo,
        spawn_entity,
        joint_state_broadcaster_spawner,
        robot_controller_spawner,
    ]

    return LaunchDescription(nodes)