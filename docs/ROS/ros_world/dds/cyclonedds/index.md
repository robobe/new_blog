---
tags:
    - cyclonedds
    - dds
---


# Cyclone DDS
[CycloneDDS web site](https://cyclonedds.io/)

## install

```
sudo apt install ros-humble-rmw-cyclonedds-cpp
```

## System configuration recommendation
[ros2 rmw cyclone settings](https://github.com/ros2/rmw_cyclonedds?tab=readme-ov-file#performance-recommendations)

- net.core.rmem_max
- net.core.rmem_default

```bash
# Add settings to sysctl 
echo "net.core.rmem_max=8388608\nnet.core.rmem_default=8388608\n" | sudo tee /etc/sysctl.d/60-cyclonedds.conf
```

```
export CYCLONEDDS_URI=file://$PWD/cyclonedds.xml
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

---

## Posts

- [cyclonedds_multicast](cyclonedds_multicast.md)
- [cyclone shm](cyclinedds_shm.md)