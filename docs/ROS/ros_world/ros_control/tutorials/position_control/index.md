---
tags:
    - ros
    - control
    - ros2_control
    - position
---

# ROS2 Control: Position control



[ROS2 control docs](https://control.ros.org/jazzy/doc/ros2_controllers/position_controllers/doc/userdoc.html)


## Gazebo simulation

!!! note "controller manager"
    The controller manager loaded by gazebo

    ```xml title="urdf/xacro"
    <robot xmlns:xacro="http://www.ros.org/wiki/xacro">
        <gazebo>
            <plugin filename="gz_ros2_control-system" name="gz_ros2_control::GazeboSimROS2ControlPlugin">
                <parameters>
                    $(find tutorial_bringup)/config/robot_controller.yaml
                </parameters>
            </plugin>
        </gazebo>
    </robot>
    ```
     
    Once gazebo launch it's load the control plugin and parse the `<ros2_control>` block.

    - Init the hardware interface
    - Launch the controller manager


```xml title="xacro loaf gazebo control plugin and set ros2_control tag"
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" >
    <gazebo>
        <plugin filename="gz_ros2_control-system" name="gz_ros2_control::GazeboSimROS2ControlPlugin">
            <parameters>
                $(find tutorial_bringup)/config/position_controllers.yaml
            </parameters>
        </plugin>

    </gazebo>
    
    <ros2_control name="robot" type="system">
        <hardware>
            <plugin>gz_ros2_control/GazeboSimSystem</plugin>
        </hardware>

        <joint name="first_joint">
            <command_interface name="position">
                <param name="min">-1.571</param>
                <param name="max">1.571</param>
            </command_interface>

            <state_interface name="position" />
        </joint>

    </ros2_control>
</robot>
```

<details>
    <summary>controllers yaml file</summary>

```yaml
--8<-- "docs/ROS/ros_world/ros_control/tutorials/position_control/code/position_controllers.yaml"
```

</details>


<details>
    <summary>launch file</summary>

```python
--8<-- "docs/ROS/ros_world/ros_control/tutorials/position_control/code/position_control.launch.py"
```

</details>


### usage

```bash title="send command"
ros2 topic pub -1 /position_controller/commands std_msgs/msg/Float64MultiArray "{ data: [1.0] }"
```

#### State
State define in ros2_control

```xml
<state_interface name="position" />
```

```bash title="echo state"
ros2 topic echo /dynamic_joint_states
#
---
header:
  stamp:
    sec: 167
    nanosec: 299000000
  frame_id: ''
joint_names:
- first_joint
interface_values:
- interface_names:
  - position
  values:
  - 1.000000000000095
---
```

!!! tip "Extent the state interface"

    Add state to `ros2_control` section
    ```bash
    <state_interface name="position" />
    <state_interface name="velocity" />
    ```

    ```bash
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
     