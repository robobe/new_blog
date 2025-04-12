---
tags:
    - gazebo
    - harmonic
    - sensors
    - camera
---

# Camera Sensor

!!! tip "Don't forget"
    ```xml title="add sensor pligin to world"
    <plugin
      filename="gz-sim-sensors-system"
      name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    ```


## RGB Camera

```xml
 <sensor name="camera" type="camera">
    <pose> 0 0 0 0 0 0 </pose>
    <visualize>true</visualize>
    <update_rate>10</update_rate>
    <camera>
        <camera_info_topic>camera/camera_info</camera_info_topic>
        <horizontal_fov>1.089</horizontal_fov>
        <image>
            <format>R8G8B8</format>
            <width>640</width>
            <height>480</height>
        </image>
        <clip>
            <near>0.05</near>
            <far>8.0</far>
        </clip>
    </camera>
    <topic>camera/image_raw</topic>
    <gz_frame_id>camera_link_optical</gz_frame_id>
</sensor>
```     

