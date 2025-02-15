---
tags:
    - dds
    - cyclonedds
    - shm
---

# CycloneDDS with SHM


Using cycloneDDS using shm

!!! note to watch
        
    [Cyclone DDS Shared Memory - Boost Data Transfer Speed and Maximize Efficiency](https://www.youtube.com/watch?v=5GpROveP6Hg)



```xml
--8<-- "docs/ROS/ros_world/dds/cyclonedds/shm.xml"
```

## iox-roudi
[roudi config](https://iceoryx.io/latest/advanced/configuration-guide/)
```bash
iox-roudi 
```

## Demo
Run tmuxp script that run `iox-roudi` server and publish image using gscam with two subscribers
We can see using bmon or other network sniffer that is no traffic

```bash title="environment variables"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///cyclonedds.xml
```

<details>
<summary>tmux script</summary>
```yaml
--8<-- "docs/ROS/ros_world/dds/cyclonedds/tmuxp_shm.yaml"
```
</details>

- [cyclonedd.xml](shm.xml)
- [tmuxp yaml to run the demo](tmuxp_shm.yaml)


---

## iceoryx-introspection
```
ros-humble-iceoryx-introspection
```

```
iox-introspection-client --h
```

--- 

## reference
- [Using Shared Memory with ROS 2](https://github.com/ros2/rmw_cyclonedds/blob/humble/shared_memory_support.md)