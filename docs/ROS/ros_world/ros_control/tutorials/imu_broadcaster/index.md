---
tags:
    - ros
    - ros2_control
    - imu
    - imu_broadcaster
---

# ROS2 Control imu broadcaster


## Gazebo simulation

```xml title="urdf/gazebo.xacro"
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
    <gazebo>
        <plugin filename="gz_ros2_control-system" name="gz_ros2_control::GazeboSimROS2ControlPlugin">
            <parameters>
                $(find tutorial_bringup)/config/robot_controller.yaml
            </parameters>
        </plugin>

    </gazebo>

    <gazebo reference="second_link">
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

```yaml title="config/controllers.yaml"
controller_manager:
  ros__parameters:
    update_rate: 10

    imu_broadcaster:
      type: imu_sensor_broadcaster/IMUSensorBroadcaster

imu_broadcaster:
  ros__parameters:

    sensor_name: link2_imu
    frame_id: second_link
```

```bash title="load controller"
ros2 control load_controller --set-state active imu_broadcaster
```

```python title="launch"
imu_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["imu_broadcaster"],
        output="screen",
    )
```

---

## Reference
- [mpu6050-ros2-control](https://github.com/TheNoobInventor/mpu6050-ros2-control/tree/main)