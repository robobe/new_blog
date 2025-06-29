---
title: Gazebo ROS IMU Bridge
tags:
    - ros
    - gazebo
    - bridge
    - jazzy
    - harmonic
    - imu
---
{{ page_folder_links() }}

Bridge IMU sensor from gazebo to ros

## Gazebo

!!! tip "Add imu plugin to world"
    ```xml title="add imu plugin to world"
    <plugin
      filename="gz-sim-imu-system"
      name="gz::sim::systems::Imu">
    </plugin>
    ```


```xml title="sensor"
<sensor name="imu" type="imu">
    <always_on>1</always_on>
    <update_rate>50</update_rate>
    <visualize>true</visualize>
    <topic>imu</topic>
    <enable_metrics>true</enable_metrics>
</sensor>
```

!!! tip "urdf"
    Don't forget to shroud with `<gazebo reference="link name"> ` tag



```bash title="gz"
gz topic --echo -t /imu
```

---

## ROS

### cli

```bash
ros2 run ros_gz_bridge parameter_bridge /imu@sensor_msgs/msg/Imu[gz.msgs.IMU
#
 Creating GZ->ROS Bridge: [/imu (gz.msgs.IMU) -> /imu (sensor_msgs/msg/Imu)] (Lazy 0)
```

### bridge file

```yaml
- ros_topic_name: "/imu"
  gz_topic_name: "/imu"
  ros_type_name: "sensor_msgs/msg/Imu"
  gz_type_name: "gz.msgs.IMU"
  direction: GZ_TO_ROS
```

### launch

TODO: check this coda again

```python title="imu_bridge.launch.py"
--8<-- "docs/ROS/ros_eco/urdf_xacro_gz_plugin/gazebo_harmonic/jazzy_bridge/imu/code/imu_bridge.launch.py"
```