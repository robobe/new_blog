---
title: Pybullet change ground texture
tags:
    - pybullet
    - texture
---


{{ page_folder_links() }}


[tarmac.png](http://models.gazebosim.org/asphalt_plane/materials/textures/tarmac.png){:target="_blank"}


```python
import pybullet as p
import pybullet_data
import time

# Initialize PyBullet
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Create ground plane
planeId = p.loadURDF("plane.urdf")

# Add texture to the ground
textureId = p.loadTexture("tarmac.png")
p.changeVisualShape(planeId, -1, textureUniqueId=textureId)

# Set gravity and run simulation
p.setGravity(0, 0, -9.81)

while True:
    p.stepSimulation()
    time.sleep(1./240.)
```