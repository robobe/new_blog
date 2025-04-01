---
tags:
    - ros
    - docker
    - jetson
    - build
    - arm
---

# ROS2 humble docker for Jetson

## Docker image
### Build
<details>
    <summary>Dockerfile</summary>

Base on ubuntu 22.04 without any nvidia support

```dockerfile
--8<-- "docs/ROS/dev_environment/build/code/Dockerfile"
```
</details>

```bash title="build"
docker buildx build --platform linux/arm64 -t humble/arm:build -f Dockerfile .
```

### Usage

```bash title="check"
docker run -it --rm \
--platform linux/arm64 \
--net host \
--hostname ros \
--user user \
humble/arm:build uname -a
```

---


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