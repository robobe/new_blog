---
tags:
    - zenoh
    - bridge
    - ros
    - shm
---

# Zenoh Bridge with SHM support

# under construction


- zenoh-plugin-ros2dds

## build with SHM
- Download source (v 1.2.0) [download](https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds/archive/refs/tags/1.2.0.zip)
- Extract and run:

```
cargo build --release --features dds_shm
```

!!! note ""
     binary locate at `target/release` folder


### Usage
- Run with shm flag
- Set cyclondds.xml for shm

```bash
./zenoh-bridge-ros2dds --dds-enable-shm
```

```xml title="cyclondds.xml"
--8<-- "docs/ROS/zenoh/ros_bridge/shm.xml"
```


#### tmux script 

```yaml
--8<-- "docs/ROS/zenoh/ros_bridge/tmuxp_shm.yaml"
```
     

---

## Reference
- [combine zenoh-plugin-dds with cyclone iceoryx shared memory #50]()