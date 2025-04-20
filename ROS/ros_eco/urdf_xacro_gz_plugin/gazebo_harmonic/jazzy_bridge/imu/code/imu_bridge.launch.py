from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from pathlib import Path

PKG_BRINGUP = 'turtlebot_bringup'
BRIDGE_CONFIG = "gz_bridge.yaml"
CONFIG_FOLDER = "config"

def generate_launch_description():
    ld = LaunchDescription()

    bridge_file = Path(get_package_share_directory(PKG_BRINGUP)) \
        .joinpath(CONFIG_FOLDER) \
        .joinpath(BRIDGE_CONFIG) \
        .as_posix()

    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args',
            '-p',
            f"config_file:={bridge_file}"
        ],
        parameters=[
            {'use_sim_time': True},
            {'qos_overrides./imu.publisher.reliability': 'best_effort'}
        ],
    )

    ld.add_action(ros_gz_bridge)


    return ld