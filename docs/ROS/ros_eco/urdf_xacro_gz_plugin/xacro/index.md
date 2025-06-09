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
- [arg](xacro_arg_and_condition.md)
- [include]()
- [condition](xacro_arg_and_condition.md)
- [macro]()
- [loops]()


---

## Recommend xacro layout

- robot.urdf.xacro: main xacro file
- gazebo.xacro: all gazebo stuff
- control.xacro: ros2 control
- materials.xacro
- macros.xacro
- inertial.xacro


```xml title="robot.urdf.xacro"
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="my_robot">
  <xacro:include filename="gazebo.xacro"/>
  <xacro:include filename="control.xacro"/>

  <xacro:property name="a" value="0.1" />
</robot>
```

```xml title="gazebo.xacro"
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <gazebo reference="imu_link">
        <sensor name="link2_imu" type="imu">
            <always_on>1</always_on>
            <update_rate>50</update_rate>
            <visualize>true</visualize>
            <topic>imu-control</topic>
            <enable_metrics>true</enable_metrics>
        </sensor>
    </gazebo>

  
</robot>
```

!!! tip "<gazebo> tag"
     

```xml title="materials.xacro"
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <material name="white">
        <color rgba="1 1 1 1"/>
    </material>

    <material name="orange">
        <color rgba="1 0.3 0.1 1"/>
    </material>

    <material name="blue">
        <color rgba="0.2 0.2 1 1"/>
    </material>
  
</robot>
```

```xml title="inertial.xacro"
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="inertial_sphere" params="mass radius *origin">
        <inertial>
            <xacro:insert_block name="origin"/>
            <mass value="${mass}" />
            <inertia ixx="${(2/5) * mass * (radius*radius)}" ixy="0.0" ixz="0.0"
                    iyy="${(2/5) * mass * (radius*radius)}" iyz="0.0"
                    izz="${(2/5) * mass * (radius*radius)}" />
        </inertial>
    </xacro:macro>  


    <xacro:macro name="inertial_box" params="mass x y z *origin">
        <inertial>
            <xacro:insert_block name="origin"/>
            <mass value="${mass}" />
            <inertia ixx="${(1/12) * mass * (y*y+z*z)}" ixy="0.0" ixz="0.0"
                    iyy="${(1/12) * mass * (x*x+z*z)}" iyz="0.0"
                    izz="${(1/12) * mass * (x*x+y*y)}" />
        </inertial>
    </xacro:macro>


    <xacro:macro name="inertial_cylinder" params="mass length radius *origin">
        <inertial>
            <xacro:insert_block name="origin"/>
            <mass value="${mass}" />
            <inertia ixx="${(1/12) * mass * (3*radius*radius + length*length)}" ixy="0.0" ixz="0.0"
                    iyy="${(1/12) * mass * (3*radius*radius + length*length)}" iyz="0.0"
                    izz="${(1/2) * mass * (radius*radius)}" />
        </inertial>
    </xacro:macro>
</robot>
```

---

## Reference
- [articulatedrobotics - Describing robots with URDF](https://articulatedrobotics.xyz/tutorials/ready-for-ros/urdf/)