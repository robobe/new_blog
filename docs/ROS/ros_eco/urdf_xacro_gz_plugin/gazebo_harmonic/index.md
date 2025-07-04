---
tags:
    - ros
    - gazebo
    - harmonic
    - bridge
    - jazzy
---

# ROS2 Gazebo harmonic integration

<div class="grid-container">
     <div class="grid-item">
            <a href="jazzy_bridge">
            <img src="images/bridge.png"  width="150" height="150">
            <p>Bridge</p></a>
        </div>
    <div class="grid-item">
       <a href="jazzy_interface">
            <img src="images/ros_gz_interface.png"  width="150" height="150">
            <p>Interface</p></a>
    </div>
    <div class="grid-item">
        <a href="project_template">
            <img src="images/project_template.png"  width="150" height="150">
            <p>Project</p></a>
    </div>
   
   </div>


## Bridge clock from gazebo

```bash title="/clock topic"
ros2 run ros_gz_bridge parameter_bridge /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock
```

### Using launch file
```python title="launch/bridge.launch.py"
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from pathlib import Path

PKG_BRINGUP = 'robot_loc_bringup'
BRIDGE_CONFIG = "bridge.yaml"
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
```

```yaml title="config/bridge.yaml"
- ros_topic_name: "/clock"
  gz_topic_name: "/clock"
  ros_type_name: "rosgraph_msgs/msg/Clock"
  gz_type_name: "gz.msgs.Clock"
  direction: GZ_TO_ROS
```