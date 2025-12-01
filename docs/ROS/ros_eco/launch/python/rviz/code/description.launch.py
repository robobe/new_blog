from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node

PKG_DESCRIPTION = "m2wr_description"
PKG_BRINGUP = "robot_loc_bringup"
URDF_XACRO_FILE = "m2wr.xacro"
RVIZ_CONFIG = "rviz.rviz"

def generate_launch_description():
    ld =  LaunchDescription()
    
    config_file = PathJoinSubstitution([
        get_package_share_directory(PKG_BRINGUP),
        'config',
        RVIZ_CONFIG
    ])

    xacro_file = PathJoinSubstitution([
        get_package_share_directory(PKG_DESCRIPTION),
        'urdf',
        URDF_XACRO_FILE
    ])

    
    robot_description_config = Command(['xacro ', xacro_file])

    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_config, 'use_sim_time': True}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # ros2 run joint_state_publisher joint_state_publisher --ros-args --param zeros.wheel_right_joint:=0.0 --param zeros.wheel_left_joint:=1.0
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen',
        parameters=[params]
        )
    
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name="joint_state_gui",
        output='screen')
    
    # ros2 run rviz2 rviz2 -d  /workspace/src/turtlebot_bringup/config/config.rviz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', config_file]
    )

    ld.add_action(node_robot_state_publisher)
    # ld.add_action(joint_state_publisher_node)
    ld.add_action(joint_state_publisher_gui_node)
    ld.add_action(rviz_node)
    return ld
    