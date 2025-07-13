---
title: Joint State Publisher
tags:
    - gazebo
    - plugin
    - joints
---


{{ page_folder_links() }}

The JointStatePublisher system publishes joint state information for a model. [more](https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1JointStatePublisher.html#details)


```xml
<plugin
    filename="gz-sim-joint-state-publisher-system"
    name="gz::sim::systems::JointStatePublisher">
    <joint_name>joint_right_wheel</joint_name>
    <joint_name>joint_left_wheel</joint_name>
</plugin>
```

---

## Reference
- [](gz-sim8/tutorials/joint_controller.md)