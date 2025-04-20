from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument

PKG_DESCRIPTION = "turtlebot_description"
URDF_XACRO_FILE = "demo.urdf.xacro"

ARG_USE_ROS2_CONTROL = "use_ros2_control"

def generate_launch_description():
    ld =  LaunchDescription()
    
    use_ros2_control = LaunchConfiguration(ARG_USE_ROS2_CONTROL)
    use_ros2_control_arg = DeclareLaunchArgument(
            ARG_USE_ROS2_CONTROL,
            default_value='false',
            description='Use sim time if true')
    
    xacro_file = PathJoinSubstitution([
        get_package_share_directory(PKG_DESCRIPTION),
        'urdf',
        URDF_XACRO_FILE
    ])

    
    urdf = Command(['xacro ', xacro_file, " use_control:=", use_ros2_control])
        

    params = {'robot_description': urdf, 'use_sim_time': True}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    ld.add_action(use_ros2_control_arg)
    ld.add_action(node_robot_state_publisher)
    return ld
