---
tags:
    - ros
    - xacro
---

# XACRO

Xacro (XML Macros) Xacro is an XML macro language. With xacro, you can construct shorter and more readable XML files by using macros that expand to larger XML expressions.
[more](http://wiki.ros.org/xacro)


## Install ROS support

```bash 
sudo apt install ros-humble-xacro
```

---

## Usage
### cli

```bash
xacro hello.urdf.xacro > hello.urdf
```

### launch file
- Load and procee xacro file
- Run `robot_state_publisher` to publish `robot_description` topic
- Run `joint_state_publisher_gui` to publish `TF's`
- Run `rviz` to view the robot


```python title="load xacro and view in rviz"
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
import xacro

PKG_BRINGUP = "self_balancing_bringup"
PKG_DESCRIPTION = "self_balancing_description"
ROBOT = "robot.urdf"

URDF = "urdf"
CONFIG = "config"


def generate_launch_description():
    ld = LaunchDescription()

    pkg_path = os.path.join(get_package_share_directory(PKG_DESCRIPTION))
    xacro_file = os.path.join(pkg_path, URDF, ROBOT)
    robot_description_config = xacro.process_file(xacro_file).toxml()
    params = {"robot_description": robot_description_config, "use_sim_time": True}

    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[params],
    )

    node_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        output="screen",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=[
            "-d",
            PathJoinSubstitution(
                [get_package_share_directory(PKG_BRINGUP), CONFIG, "display.rviz"]
            ),
        ],
    )

    ld.add_action(node_robot_state_publisher)
    ld.add_action(node_state_publisher_gui)
    ld.add_action(rviz_node)
    return ld

```

---

## XACRO terms

- [match]()
- [property]()
- [arg](#arg)
- [include]()
- [condition]()
- [macro]()
- [loops]()


### arg
Simple `xacro` file that get argument from outside.  
if no argument set it use the default.  
Using `xacro` command to substitute xacro sentence

```xml title="simple xacro"
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="robot_name">
    <xacro:arg name="name" default="default_bot"/>

    <link name="$(arg name)">
        
    </link>
  

</robot>
```

```bash title="usage"
# use default value
xacro demo.urdf.xacro

# set arg from cli
xacro demo.urdf.xacro name:=turtle

# convert to urdf file
xacro demo.urdf.xacro name:=turtle > demo.urdf
```

#### launch file
- Use python pathlib for path construct
- Use xacro library
- Check [condition](#condition) example for command and PathJoinSubstitution 

```python
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro
from pathlib import Path

PKG_DESCRIPTION = "turtlebot_description"
URDF_XACRO_FILE = "demo.urdf.xacro"


def generate_launch_description():
    ld =  LaunchDescription()
    
    xacro_file = Path(
        get_package_share_directory(PKG_DESCRIPTION)) \
        .joinpath("urdf") \
        .joinpath(URDF_XACRO_FILE) \
        .as_posix()
        

    urdf = xacro.process_file(xacro_file, mappings={"name":"my_name"}).toxml()
    params = {'robot_description': urdf, 'use_sim_time': True}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )


    ld.add_action(node_robot_state_publisher)
    return ld

```

```bash title="usage"
#-f outout topic full data
ros2 topic echo /robot_description -f
```

---

### Condition

```xml title="simple xacro with condition and argument"
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="robot_name">
    <xacro:arg name="use_control" default="false"/>

    <xacro:if value="$(arg use_control)">
        <!-- ros2 control-->
         <link name="ros2_control">
            
         </link>
    </xacro:if>
    <xacro:unless value="$(arg use_control)">
        <!-- other plugin-->
        <link name="ros2">
            
        </link>
    </xacro:unless>
  

</robot>
```

#### usage
Launch file with argument to control urdf parse (use ros2_control or not)
using substitutions class :

- LaunchConfiguration
- Command
- PathJoinSubstitution

<details>
    <summary>launch file that set xacro argument</summary>

```python
--8<-- "docs/ROS/ros_eco/urdf_xacro_gz_plugin/xacro/code/xacro_arg_condition.launch.py"
```
</details>

```bash title="how to launch"
# with
ros2 launch turtlebot_bringup demo.launch.py use_ros2_control:=True
# Without
ros2 launch turtlebot_bringup demo.launch.py use_ros2_control:=False
```

```bash title="echo topic"
# -f: output all topic data
ros2 topic echo /robot_description -f
```