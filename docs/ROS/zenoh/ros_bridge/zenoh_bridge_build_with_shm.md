---
tags:
    - zenoh
    - bridge
    - ros
    - shm
---

# Zenoh Bridge with SHM support

- zenoh-plugin-ros2dds

## build with SHM
- Clone [zenoh-plugin-ros2dds]()
- Clone [iceoryx]()

!!! warning "fix include"
    Create symbolink link between `/usr/include` to build location

    ```
    cd /usr/include
    sudo ls -s <>/include/iceoryx_binding_c /iceoryx_binding_c
    ```


### Build

```
cargo build --releases --features dds_shm
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