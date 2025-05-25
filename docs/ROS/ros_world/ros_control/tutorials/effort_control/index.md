---
tags:
    - ros
    - control
    - ros2_control
    - effort
---

# ROS2 Control: effort control

In ROS 2 Control, the **effort_controllers/JointGroupEffortController** is used to send effort (torque/force) commands to joints


```xml title="ros2_control urdf"
<ros2_control name="robot" type="system">
        <hardware>
            <plugin>gz_ros2_control/GazeboSimSystem</plugin>
        </hardware>

        <joint name="first_joint">
            <command_interface name="effort">
                <param name="min">-2.0</param>
                <param name="max">2.0</param>
            </command_interface>

            <state_interface name="position" />
        </joint>

    </ros2_control>
```

```yaml title="controllers.yaml"
controller_manager:
  ros__parameters:
    update_rate: 10

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    effort_controller:
      type: effort_controllers/JointGroupEffortController

effort_controller:
  ros__parameters:
    joints:
      - first_joint

```

!!! note "controller manager and gazebo"
    The controller manager loaded by gazebo automatically
    we need to bridge to gazebo clock using **ros_gz_bridge**
     

```python title="clock bridge"
gazebo_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="parameter_bridge",
        output="screen",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
    )
```

```python title="load controller"
effort_control_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["effort_controller"],
        output="screen",
    )

joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

```

---

## Usage

```bash
ros2 topic pub -1 /effort_controller/commands std_msgs/msg/Float64MultiArray "{ data: [-2.0] }"
```