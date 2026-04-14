---
title: RGB Camera sensor
tags:
    - gazebo
    - harmonic
    - sensors
    - camera
---


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
--8<-- "docs/Simulation/Gazebo/sensors/camera/code/rgb_camera_sensor.xml"
```     

---

## Demo:

<details>
<summary>World with camera</summary>
```
--8<-- "docs/Simulation/Gazebo/sensors/camera/code/camera_world.sdf"
```
</details>


![alt text](image/camera_world.png)


!!! tip "Add image display"
    

---

## Demo: Simple code to view the image using opencv and gz-transport

```python
import numpy as np
import cv2
import time

from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image

def on_image(msg: Image):
    # Get image size
    width = msg.width
    height = msg.height

    # Convert raw bytes to numpy
    img = np.frombuffer(msg.data, dtype=np.uint8)

    # Most Gazebo cameras use RGB8
    img = img.reshape((height, width, 3))

    # Convert RGB → BGR for OpenCV
    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

    # Show image
    cv2.imshow("Gazebo Camera", img)
    cv2.waitKey(1)

node = Node()

ok = node.subscribe(Image, "/camera", on_image)
if not ok:
    raise RuntimeError("Failed to subscribe")

print("Listening to camera...")

# Keep program alive
while True:
    time.sleep(1)
```