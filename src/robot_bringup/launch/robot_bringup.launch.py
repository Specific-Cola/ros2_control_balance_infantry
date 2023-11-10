import os
from ament_index_python.packages import get_package_share_directory
 
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

def generate_launch_description():
    package_name ='robot_description'
    
    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch','robot.launch.py'
        )]),launch_arguments={'use_sim_time': 'true'}.items()
        
    )
    
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'),'launch','gazebo.launch.py')]),
    )
    
    spawn_entity = Node(package='gazebo_ros',executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity','robot'],
                        output='screen'
    )
    
    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_state"],
    )
    
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["diff_drive"],
        
    )
    
    return LaunchDescription([
        robot,
        gazebo,
        spawn_entity,
        diff_drive_spawner,
        jsb_spawner,
    ])