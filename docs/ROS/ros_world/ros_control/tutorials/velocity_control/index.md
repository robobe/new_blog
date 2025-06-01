---
tags:
    - ros
    - control
    - ros2_control
    - velocity
---

# ROS2 Control: velocity control

## Gazebo simulation


```xml title="xacro loaf gazebo control plugin and set ros2_control tag"
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" >
    <gazebo>
        <plugin filename="gz_ros2_control-system" name="gz_ros2_control::GazeboSimROS2ControlPlugin">
            <parameters>
                $(find tutorial_bringup)/config/velocity_controllers.yaml
            </parameters>
        </plugin>

    </gazebo>
    
    <ros2_control name="robot" type="system">
        <hardware>
            <plugin>gz_ros2_control/GazeboSimSystem</plugin>
        </hardware>

        <joint name="first_joint">
            <command_interface name="velocity">
                <param name="min">-0.5</param>
                <param name="max">0.5</param>
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
--8<-- "docs/ROS/ros_world/ros_control/tutorials/velocity_control/code/velocity_controllers.yaml"
```

</details>


<details>
    <summary>launch file</summary>

```python
--8<-- "docs/ROS/ros_world/ros_control/tutorials/velocity_control/code/velocity_control.launch.py"
```

</details>


### usage

```bash title="send command"
ros2 topic pub -1 /velocity_controller/commands std_msgs/msg/Float64MultiArray "{ data: [1.0] }"
```

```bash
ros2 topic echo /dynamic_joint_states
#
header:
stamp:
    sec: 85
    nanosec: 799000000
frame_id: ''
joint_names:
- first_joint
interface_values:
- interface_names:
- position
- velocity
values:
- 1.00
- 0
```