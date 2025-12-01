import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, Command
from launch_ros.actions import Node

WORLD = "my_world.sdf"

PKG_DESCRIPTION = "m2wr_description"
PKG_BRINGUP = "robot_loc_bringup"
URDF_XACRO_FILE = "m2wr.xacro"
PKG_GAZEBO = "robot_loc_gazebo"

def generate_launch_description():
    ld = LaunchDescription()

    
    xacro_file = PathJoinSubstitution([
        get_package_share_directory(PKG_DESCRIPTION),
        'urdf',
        URDF_XACRO_FILE
    ])
    

    
    robot_description_config = Command(['xacro ', xacro_file, " prefix:='ns/'"])
    params = {'robot_description': robot_description_config, 'use_sim_time': True}

    gz_args = " ".join([
        "-r",
        "-v4",
        WORLD
    ])

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
                    launch_arguments={
                        'gz_args': gz_args,
                        "on_exit_shutdown": "true"}.items()
             )

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    spawn_entity = Node(package='ros_gz_sim', executable='create',
                        arguments=['-topic', 'robot_description',
                                   '-name', 'my_bot',
                                   '-z', '0.2'],
                        output='screen')
    
    ld.add_action(node_robot_state_publisher)
    ld.add_action(gazebo)
    ld.add_action(spawn_entity)
    return ld