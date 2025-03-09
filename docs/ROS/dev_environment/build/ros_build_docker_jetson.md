---
tags:
    - ros
    - docker
    - jetson
    - build
---

# ROS2 humble docker for Jetson

## Docker image
### Build
<details>
    <summary>Dockerfile</summary>

```dockerfile
--8<-- "docs/ROS/dev_environment/build/Dockerfile.jetson"
```
</details>

### Usage

```bash
docker run -it --rm \
--net host \
--hostname ros \
--user user \
humble:ver1 /bin/bash
```

# TODO
- support cuda
- support opencv
- support tensorrt
- support pytorch / tensorflow
---

## Reference
- [Dusty - Machine Learning Containers for Jetson and JetPack](https://github.com/dusty-nv/jetson-containers/tree/master)
- [atinfinity](https://github.com/atinfinity/l4t-ros2-docker/tree/main)
- [ROS 2 on Jetson Nano using Docker](https://www.codetomotion.com/blog/ros-2-on-jetson-nano-using-docker/)