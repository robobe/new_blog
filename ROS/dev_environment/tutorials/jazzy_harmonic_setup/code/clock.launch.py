import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PACKAGE_NAME = 'gz_tutorial'

def generate_launch_description():
    ld = LaunchDescription()

    bridge_params = os.path.join(get_package_share_directory(PACKAGE_NAME),'config','gz_bridge.yaml')
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ],
        parameters=[
            {'use_sim_time': True},
        ],
    )

    ld.add_action(ros_gz_bridge)


    return ld
    
