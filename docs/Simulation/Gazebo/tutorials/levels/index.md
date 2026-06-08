---
title: Gazebo levels
tags:
    - gazebo
    - levels
---


---

## Demo

```bash title="run gz with levels"
gz sim levels.sdf --levels
```

```bash
gz topic -t "/model/vehicle_blue/cmd_vel" -m gz.msgs.Twist -p "linear: {x: 4.0}"
gz topic -t "/model/vehicle_red/cmd_vel" -m gz.msgs.Twist -p "linear: {x: 2.0}"
```

<a href="code/levels.sdf" download="levels.sdf">Download code</a>

---

## Reference
- [gazebo tutorial levels](https://gazebosim.org/api/gazebo/3/levels.html)
- [tunnel world with levels](https://github.com/gazebosim/gz-sim/blob/main/examples/worlds/tunnel.sdf)
