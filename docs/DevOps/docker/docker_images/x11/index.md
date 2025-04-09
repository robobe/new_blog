---
tags:
    - docker
    - x11
    - gui
    - devices
---

# X11 Docker Images

Create docker images for X11 GUI applications. base on ubunu:22.04 

<details>
    <summary>Dockerfile</summary>

```Dockerfile title="dockerfile"
--8<-- "docs/DevOps/docker/docker_images/x11/code/Dockerfile"
```
</details>

```bash title="build"
docker build -t ubuntu/22.04:gui -f Dockerfile .
```

The command **xhost +local:docker** is used on Linux systems to manage access control for the X11 display server.
Allows any process running locally as part of the "docker" context (Docker containers) to connect to your X server.

```bash
xhost +local:docker
```

!!! tip "check x11 app"
    **glxgears** is a simple OpenGL application that draws a rotating set of gears. It is often used to test the performance of OpenGL on a system.
     
```bash title="run"
docker run -it --rm \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    ubuntu/22.04:gui \
    glxgears
```

## Share camera with docker

### Simple
Device **must** be connected before running the container.

```bash title="run"
docker run -it --rm \
    --user user \
    --hostname gui \
    --device /dev/video0:/dev/video0 \
    -e DISPLAY=$DISPLAY \
    --user user \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v /run/user/1000:/run/user/1000 \
    ubuntu/22.04:gui \
    /bin/bash
```

```bash
# Test
gst-launch-1.0 videotestsrc ! videoconvert ! autovideosink

# v4l
gst-launch-1.0 v4l2src device=/dev/video0 ! videoconvert ! autovideosink
```
### Advanced
```bash title="run"
docker run -it --rm \
    --user user \
    --hostname gui \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v /dev/dri:/dev/dri \
    --device-cgroup-rule='c 81:* rwm' \
    -v /dev:/dev \
    ubuntu/22.04:gui \
    /bin/bash
```