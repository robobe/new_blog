---
title: JointPositionController plugin
tags:
    - gazebo
    - plugin
    - joint 
    - position
---

JointPositionController uses a PID controller to reach a desired joint position.

```xml title="joint position control plugin"
<plugin
 filename="gz-sim-joint-position-controller-system"
 name="gz::sim::systems::JointPositionController">
    <joint_name>j1</joint_name>
    <topic>topic_name</topic>
    <p_gain>1</p_gain>
    <i_gain>0.1</i_gain>
    <d_gain>0.01</d_gain>
    <i_max>1</i_max>
    <i_min>-1</i_min>
    <cmd_max>1000</cmd_max>
    <cmd_min>-1000</cmd_min>
</plugin>
```


```bash title="cli command"
gz topic -t "/topic_name" -m gz.msgs.Double -p "data: -1.0"
```

## Demo: Control joint position


```bash
gz topic -t "/joint2_position" -m gz.msgs.Double -p "data: -1.0"
```

---

## Reference
- [gazebo docs: Joint Controllers](https://gazebosim.org/api/sim/10/jointcontrollers.html)