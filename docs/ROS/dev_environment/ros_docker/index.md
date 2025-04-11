---
tags:
    - ros2
    - docker
---

# Using Docker for ROS2 Development
ROS images use as a base line for project development.


## Images

### Humble base
- ros-base
- build tools
- ros build tools
- basic system utilities
- add none-root user

<details>
    <summary>Dockerfile ROS humble </summary>

```dockerfile
--8<-- "docs/ROS/dev_environment/ros_docker/code/Dockerfile.humble"
```
</details>

#### Downloads
- [Dockerfile](code/Dockerfile.humble)
- [tmux config](code/.tmux.conf)

```bash title="Build image"
docker build -t humble:base -f Dockerfile.humble .
```

---

## Docker and Devices
How to run docker with devices like camera joysticks and other peripherals.

- [Must watch and read](https://articulatedrobotics.xyz/tutorials/docker/devices-docker/)

---

## Nvidia

- [l4t-ros2-docker](https://github.com/atinfinity/l4t-ros2-docker)