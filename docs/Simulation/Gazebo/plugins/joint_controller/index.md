---
title: JointController plugin
tags:
    - gazebo
    - plugin
    - joint 
    - velocity
    - force
---

[gazebo source demo world](https://github.com/gazebosim/gz-sim/blob/main/examples/worlds/joint_controller.sdf)


- **Velocity mode**: This mode lets the user control the desired joint velocity **directly**.
- **Force mode**: A user who wants to control joint **velocity** using a PID controller .

```xml title="velocity mode"
<plugin
 filename="gz-sim-joint-controller-system"
 name="gz::sim::systems::JointController">
 <joint_name>j1</joint_name>
</plugin>
```

create topic named

```xml title="default topic name"
/model/<model_name>/joint/<joint_name>/cmd_vel
```

### force mode
Still ask for **velocity** but now the velocity control by apply force

[Joint controllers PID settings ](https://gazebosim.org/api/sim/10/classgz_1_1sim_1_1systems_1_1JointController.html)

!!! warning "PID settings"
    If the pid not config properly  the joint controller failed to reach the desired velocity and behaved absurdly due to improper gains  

```xml title="force mode"
<plugin
 filename="gz-sim-joint-controller-system"
 name="gz::sim::systems::JointController">
 <joint_name>j1</joint_name>
 <use_force_commands>true</use_force_commands>
 <p_gain>0.2</p_gain>
 <i_gain>0.01</i_gain>
</plugin>
```

---

## Demo: Velocity control

<details>
<summary>Velocity</summary>
```
--8<-- "docs/Simulation/Gazebo/plugins/joint_controller/code/joint_controller_vel.sdf"
```
</details>



---

## Reference
- [gazebo sim - Joint Controllers](https://gazebosim.org/api/sim/10/jointcontrollers.html)
- [gazebo API - Joint controllers](https://gazebosim.org/api/sim/10/classgz_1_1sim_1_1systems_1_1JointController.html)