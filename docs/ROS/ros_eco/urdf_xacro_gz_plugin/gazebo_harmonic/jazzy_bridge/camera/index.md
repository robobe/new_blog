---
tags:
    - ros
    - gazebo
    - bridge
    - jazzy
    - harmonic
    - camera
---

# Camera
Bridge rgb camera between gazebo to ros

## Gazebo

!!! tip "Add sensor plugin to world"
    ```xml title="add sensor plugin to world"
    <plugin
      filename="gz-sim-sensors-system"
      name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    ```
     

```xml title="RGB Camera"
--8<-- "docs/Simulation/Gazebo/sensors/code/rgb_camera_sensor.xml"
```  

!!! tip "urdf"
    Don't forget to shroud with `<gazebo reference="link name"> ` tag


     
[download world](code/camera_world.sdf)

---

## ROS2
[check ros gz sim demo](https://github.com/gazebosim/ros_gz/tree/jazzy/ros_gz_sim_demos#camera)

### ros_gz_bridge

```bash
ros2 run ros_gz_bridge parameter_bridge /camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image
```

#### Using launch file
- load bridge config from yaml file
- control topic qos


```python title="ros_gz_bridge launch file"
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from ros_gz_bridge.actions import RosGzBridge

PACKAGE_NAME = 'tutorial_bringup'

def generate_launch_description():
    ld = LaunchDescription()

    bridge_params = os.path.join(get_package_share_directory(PACKAGE_NAME),'config','gz_bridge.yaml')
    bridge = RosGzBridge(
            bridge_name="bridge",
            config_file=bridge_params,
        )

    ld.add_action(bridge)


    return ld
```

```yaml title="gz_bridge.yaml"
- ros_topic_name: "/camera/image_raw"
  gz_topic_name: "/camera/image_raw"
  ros_type_name: "sensor_msgs/msg/Image"
  gz_type_name: "gz.msgs.Image"
  direction: GZ_TO_ROS

```

<!-- !!! tip "control image qos"
    Add to parameters section

    ```json
    {'qos_overrides./camera/image_raw.publisher.reliability': 'best_effort'}
    ```
     
 -->



---

## ros_gz_image

use [image_transport](http://wiki.ros.org/image_transport)

```bash title="image transport bridge "
sudo apt install ros-jazzy-ros-gz-image
```

```bash title=""
ros2 run ros_gz_image image_bridge /camera/image_raw
```

---

### rqt

```bash
ros2 run rqt_image_view rqt_image_view /camera/image_raw
```

![alt text](images/ret_image_view.png)
