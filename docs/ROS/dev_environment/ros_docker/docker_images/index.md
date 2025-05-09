---
tags:
    - docker
    - ros
    - image
---

# ROS Docker Images for humble and jazzy

<div class="grid-container">
    <div class="grid-item">
        <a href="#humble">
        <img src="images/humble.png" width="150" height="150">
        <p>Humble</p>
        </a>
    </div>
    <div class="grid-item">
    <a href="#jazzy">
        <img src="images/jazzy.png" width="150" height="150">
        <p>Jazzy</p>
        </a>
    </div>
    <div class="grid-item">
        
        <p>TBD</p>
        
    </div>
    
</div>

## Humble

### Humble base
- ros-base
- build tools
- ros build tools
- basic system utilities
- add none-root user

<details>
    <summary>Dockerfile ROS humble </summary>

```dockerfile
--8<-- "docs/ROS/dev_environment/ros_docker/docker_images/code/Dockerfile.humble"
```
</details>

#### Downloads
- [Dockerfile](code/Dockerfile.humble)
- [tmux config](code/tmux.conf)

```bash title="Build image"
docker build -t humble:base -f Dockerfile.humble .
```

---

## Jazzy

### Jazzy base
- base on cuda 12.4.0 ubuntu 24.04
- ros-base
- build tools
- ros build tools
- basic system utilities
- add none-root user
- gazebo bridge (harmonic)
<details>
    <summary>Dockerfile ROS jazzy </summary>

```dockerfile
--8<-- "docs/ROS/dev_environment/ros_docker/docker_images/code/Dockerfile.jazzy"
```
</details>

!!! warning "cuda version"
    Check host cuda version using `nvidia-smi` , I encountered problems if the docker image cuda version was higher
    
     
#### Downloads
- [Dockerfile](code/Dockerfile.jazzy)
- [tmux config](code/.tmux.conf)

```bash title="Build image"
docker build -t jazzy:base -f Dockerfile.jazzy .
```

### usage

```bash
docker run --gpus all -it --rm \
--net host \
--name jazzy \
--hostname jazzy \
--user user \
jazzy:base \
/bin/bash
```