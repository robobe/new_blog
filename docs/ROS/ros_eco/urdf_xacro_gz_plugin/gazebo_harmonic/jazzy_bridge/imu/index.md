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


!!! tip "sdf imu"
  [SDF specification](http://sdformat.org/spec?ver=1.12&elem=sensor#sensor_imu)
     
```xml title="sensor"
<sensor name="imu" type="imu">
    <always_on>1</always_on>
    <update_rate>50</update_rate>
    <visualize>true</visualize>
    <topic>imu</topic>
    <enable_metrics>true</enable_metrics>
    <gz_frame_id>imu_link</gz_frame_id>
</sensor>
```

!!! tip "urdf"
    Don't forget to shroud with `<gazebo reference="link name"> ` tag

    ```xml
    <!-- link -->
    <link name="imu_link"/>

    <!-- plugin -->
    <gazebo reference="imu_link">
      <sensor name="imu" type="imu">
        <always_on>1</always_on>
        <update_rate>50</update_rate>
        <visualize>true</visualize>
        <topic>imu</topic>
        <enable_metrics>true</enable_metrics>
        <gz_frame_id>imu_link</gz_frame_id>
      </sensor>
    </gazebo>
    ```

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

!!! tip "frame_id"
    By default gazebo frame_id will be sensor link
     

    ```json
    header {
      stamp {
        sec: 901
        nsec: 340000000
      }
      data {
        key: "frame_id"
        value: "my_bot::base_footprint::imu"
      }
      data {
        key: "seq"
        value: "36688"
      }
    }
    ```

    this map to in ros message

    ```yaml
    header:
      stamp:
        sec: 61
        nanosec: 220000000
      frame_id: my_bot::base_footprint::imu
    ```

    using `<gz_frame_id>imu_link</gz_frame_id>` to control frame_id value


    ```yaml
    ros2 topic echo --once /imu --field header

    #
      stamp:
        sec: 149
        nanosec: 540000000
      frame_id: imu_link
    ```

---

