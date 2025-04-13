---
tags:
    - gazebo
    - harmonic
    - sensors
    - camera
---

# Camera Sensor

!!! tip "Don't forget"

    ```xml title="add sensor plugin to world"
    <plugin
      filename="gz-sim-sensors-system"
      name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    ```


## RGB Camera

```xml
--8<-- "docs/Simulation/Gazebo/sensors/code/rgb_camera_sensor.xml"
```     

