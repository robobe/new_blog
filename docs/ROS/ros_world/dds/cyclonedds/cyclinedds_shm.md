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
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:schemaLocation="https://cdds.io/config https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/iceoryx/etc/cyclonedds.xsd">
    <Domain id="any">
        <SharedMemory>
            <Enable>true</Enable>
            <LogLevel>info</LogLevel>
        </SharedMemory>
    </Domain>
</CycloneDDS>
```

## iox-roudi
[roudi config](https://iceoryx.io/latest/advanced/configuration-guide/)
```bash
iox-roudi 
```

## Demo
Run tmuxp script that run `iox-roudi` server and publish image using gscam with two subscribers
We can see using bmon or other network sniffer that is no traffic


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