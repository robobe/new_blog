---
title: Gazebo python bindings
tags:
tags:
    - gazebo
    - api
    - bindings
    - transport
    - python
---


## Demo:
Subscribe to joint state message using transport api

```python
import math
import threading
import time

from gz.transport13 import Node
from gz.msgs10.double_pb2 import Double
from gz.msgs10.model_pb2 import Model

q = 0.0
qdot = 0.0
have_state = False
lock = threading.Lock()

q_ref = 0.7
k1 = 12.0
k2 = 3.5
tau_max = 8.0

def on_joint_state(msg: Model):
    print(msg)

joint_state_topic_name = "/world/default/model/joint_position_controller_demo/joint_state"
node = Node()
ok = node.subscribe(Model, joint_state_topic_name, on_joint_state)
if not ok:
    raise RuntimeError("Failed to subscribe to /simple_arm/joint_state")


while True:
    time.sleep(0.005)
```