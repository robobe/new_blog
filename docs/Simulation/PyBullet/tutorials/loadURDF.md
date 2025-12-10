---
title: PyBullet tutorials and learning
tags:
    - pybullet
    - loadurdf
    - changeVisualShape
---


```python
import pybullet as p
import pybullet_data
import time

# setup
p.connect(p.GUI)
p.resetSimulation()
p.setGravity(gravX=0, gravY=0, gravZ=-9.8)
p.setAdditionalSearchPath(path=pybullet_data.getDataPath())

# load URDF
plane = p.loadURDF("plane.urdf")

cube = p.loadURDF(
    "cube.urdf", basePosition=[0, 0, 1], globalScaling=0.5, useFixedBase=True
)

obj = p.loadURDF("cube_small.urdf", basePosition=[1, 1, 0.1], globalScaling=1.5)
p.changeVisualShape(objectUniqueId=obj, linkIndex=-1, rgbaColor=[1, 0, 0, 1])  # red

p.setRealTimeSimulation(True)
while p.isConnected():
    time.sleep(0.01)

```

![alt text](image.png)

- **useFixedBase**: force the base of the loaded object to be static
- **basePosition**: create the base of the object at the specified position in
world space coordinates [X,Y,Z].
- **globalScaling**: globalScaling will apply a scale factor to the URDF
model


## changeVisualShape
[guid](../code/pybullet_quickstartguide.pdf#53)
You can use changeVisualShape to change the texture of a shape, the RGBA color and other
properties.