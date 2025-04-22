---
tags:
    - gazebo
    - harmonic
    - plugin
---

# Gazebo plugins

## diff drive

[source code](https://github.com/gazebosim/gz-sim/tree/gz-sim9/src/systems/diff_drive)

### simple gazebo declaration
```xml
--8<-- "docs/Simulation/Gazebo/plugins/diff_drive.xml"
```

[moving robot tutorial](https://github.com/gazebosim/docs/blob/master/harmonic/moving_robot.md)

```bash title="gz topic publish"
gz topic -t "/cmd_vel" -m gz.msgs.Twist -p "linear: {x: 0.5}, angular: {z: 0.0}"
```