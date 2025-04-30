import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution

PKG = "turtlesim_loc"

def generate_launch_description():
    ld = LaunchDescription()
    
    ekf_config_odom = PathJoinSubstitution([
        get_package_share_directory(PKG),
        'config',
        'ekf_odom.yaml'
    ])

    ekf_config_map = PathJoinSubstitution([
        get_package_share_directory(PKG),
        'config',
        'ekf_map.yaml'
    ])

    turtle = Node(
        package='turtlesim',
        executable='turtlesim_node',
        output='screen')

    position_sensor_emulation = Node(
        package='turtlesim_loc',
        executable='position.py',
        output='screen')

    twist_sensor_emulation = Node(
        package='turtlesim_loc',
        executable='twist.py',
        output='screen')
    
    teleop = Node(
        package='turtlesim',
        executable='turtle_teleop_key',
        output='screen')
    # ros2 run robot_localization ekf_node --ros-args --params-file /workspace/src/localiztion_demo/turtlesim_loc/config/ekf.yaml --log-level info
    ekf_node_odom = Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config_odom],
            remappings=[
                ('odometry/filtered', 'odometry/filtered_odom')
            ]
        )
    
    ekf_node_map = Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config_map],
            remappings=[
                ('odometry/filtered', 'odometry/filtered_map')
            ]
        )
    
    ld.add_action(turtle)
    ld.add_action(position_sensor_emulation)
    ld.add_action(twist_sensor_emulation)
    # ld.add_action(teleop)
    ld.add_action(ekf_node_map)
    ld.add_action(ekf_node_odom)
    return ld
