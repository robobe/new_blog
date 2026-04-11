---
title: ApplyJointForce plugin
tags:
    - gazebo
    - plugin
    - joint 
    - force
    - torque
---
ApplyJointForce is `model` plugin that
Apply joint raw force/torque to specific joint, create subscriber the names `/model/<model named>/joint/<joint name>/cmd_force` can control from cli using

```bash title="joint force using cli"
# joint_force_example is the model name
# j1 is the joint name
gz topic -t "/model/joint_force_example/joint/j1/cmd_force" -m gz.msgs.Double  -p "data: 0.001"
```

```xml title="plugin xml declaration"
<plugin
    filename="gz-sim-apply-joint-force-system"
    name="gz::sim::systems::ApplyJointForce">
        <joint_name>j1</joint_name>
</plugin>
```

---


- [Demo World](https://github.com/gazebosim/gz-sim/blob/main/examples/worlds/apply_joint_force.sdf)


---

### force
Constant force (or torque for revolute joints)
Units:

- Revolute → Nm (torque)
- Prismatic → N (linear force)

!!! tip "Don't forget joint damping"

    ```xml
    <joint name="j1" type="revolute">
        <pose>0 0 -0.5 0 0 0</pose>
        <parent>base_link</parent>
        <child>rotor</child>
        <axis>
          <xyz>0 0 1</xyz>
          <dynamics>
            <damping>0.0001</damping>
          </dynamics>
        </axis>
      </joint>
    ```
    
!!! info "torque apply to joint"
    **Not to the link**
    

---

## Reference

- [plugin source code](https://github.com/gazebosim/gz-sim/blob/main/src/systems/apply_joint_force)