---
tags:
    - ros
    - control
    - ros2_control
    - effort
---

# ROS2 Control: effort control

In ROS 2 Control, the **effort_controllers/JointGroupEffortController** is used to send effort (torque/force) commands to joints


## Gazebo simulation

```xml title="xacro load gazebo control plugin and set ros2_control tag"
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" >
    <gazebo>
        <plugin filename="gz_ros2_control-system" name="gz_ros2_control::GazeboSimROS2ControlPlugin">
            <parameters>
                $(find tutorial_bringup)/config/effort_controllers.yaml
            </parameters>
        </plugin>

    </gazebo>
    
    <ros2_control name="robot" type="system">
        <hardware>
            <plugin>gz_ros2_control/GazeboSimSystem</plugin>
        </hardware>

        <joint name="first_joint">
            <command_interface name="effort">
                <param name="min">-10</param>
                <param name="max">10</param>
            </command_interface>

            <state_interface name="position" />
            <state_interface name="velocity" />
        </joint>

    </ros2_control>
</robot>
```


<details>
    <summary>controllers yaml file</summary>

```yaml
--8<-- "docs/ROS/ros_world/ros_control/tutorials/effort_control/code/effort_controllers.yaml"
```

</details>


<details>
    <summary>launch file</summary>

```python
--8<-- "docs/ROS/ros_world/ros_control/tutorials/effort_control/code/effort_control.launch.py"
```

</details>

---

## Usage

```bash
ros2 topic pub -1 /effort_controller/commands std_msgs/msg/Float64MultiArray "{ data: [-2.0] }"
```