---
tags:
    - gazebo
    - classic
    - tips
---



## Send message to subscriber

```bash
# gz topic -p topic msg_type -m message_data
gz topic -p "/gazebo/default/iris_demo/gimbal_tilt_cmd"  "gazebo.msgs.GzString" -m 'data: "1.0"'
```


---

## Reference
- [5 Ways to Speedup Simulations](https://www.blackcoffeerobotics.com/blog/gazebo-simulator-5-ways-to-speedup-simulations)