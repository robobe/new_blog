---
title: Multi docker workspace using docker compose
tags:
    - ros
    - workspace
    - docker
    - gazebo
    - compose
---
{{ page_folder_links() }}

Run multiple docker using docker compose

## Demo
Run ROS jazzy in one docker container and gazebo harmonic in the another container

### Jazzy docker file
- Add ros-gz-bridge
- Add gazebo repo and install libgz-transport13

```dockerfile
# install gazebo transport
# transport13 for harmonic version
RUN apt-get update && apt-get install -q -y \
    curl \
    gnupg \
    lsb-release \
    python3-argcomplete \
    sudo \
    wget \
  && wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
  && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null \
  && apt-get update && apt-get install -y -q \
    libgz-transport13 \
  && rm -rf /var/lib/apt/lists/*

# install bridge
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-ros-gz-bridge \
  && rm -rf /var/lib/apt/lists/*
```

```json title="devontainer.json"
{
    "name": "multi",
    "dockerComposeFile": "../docker-compose.yaml",
    "service": "ros",
    "runServices": ["ros", "gazebo"],
}
```

```yaml title="docker-compose.yaml"
services:
  ros:
    build:
      context: .
      dockerfile: .devcontainer/Dockerfile
    user: "ros:ros"
    volumes:
      - .:/workspace:cached
      - /tmp/.X11-unix:/tmp/.X11-unix:rw 
      - /dev/dri:/dev/dri 
      - /dev/nvidia0:/dev/nvidia0 
      - /dev/nvidiactl:/dev/nvidiactl 
      - /dev/nvidia-modeset:/dev/nvidia-modeset 
    deploy:
      resources:
        reservations:
          devices:
            - capabilities: [gpu]
    hostname: ros
    networks: [sim_net]
    stdin_open: true
    tty: true
    environment:
      - DISPLAY=${DISPLAY}
      - XAUTHORITY=${XAUTHORITY}
      # - LIBGL_ALWAYS_SOFTWARE=1  # Force software rendering
      - MESA_GL_VERSION_OVERRIDE=3.3
      - QT_X11_NO_MITSHM=1
      - GZ_RENDER_ENGINE=ogre2
      - GZ_TRANSPORT_IP=gazebo
      - GZ_DISCOVERY_SERVER=gazebo
      - GZ_PARTITION=my_simulation
    devices:
      - /dev/dri:/dev/dri
  gazebo:
    build:
      context: .
      dockerfile: .devcontainer/Dockerfile.harmonic
      target: base
    networks: [sim_net]
    user: user
    stdin_open: true
    tty: true
    hostname: gz
    environment:
      - DISPLAY=${DISPLAY}
      - QT_X11_NO_MITSHM=1
      - NVIDIA_VISIBLE_DEVICES=all
      - NVIDIA_DRIVER_CAPABILITIES=all
      - GZ_TRANSPORT_IP=gazebo
      - GZ_DISCOVERY_SERVER=gazebo
      - GZ_PARTITION=my_simulation
    volumes:
      - .:/workspace:cached
      - ./.gz:/home/user/.gz:cached
      - /tmp/.X11-unix:/tmp/.X11-unix:rw 
      - /dev/dri:/dev/dri 
      - /dev/nvidia0:/dev/nvidia0 
      - /dev/nvidiactl:/dev/nvidiactl 
      - /dev/nvidia-modeset:/dev/nvidia-modeset 
    deploy:
      resources:
        reservations:
          devices:
            - capabilities: [gpu]
networks:
  sim_net:
    driver: bridge

```

### Usage
Bridge `Clock`


#### Terminal 1 (gazebo)
Get gazebo side terminal using

```bash
# from project workspace
docker compose exec gazebo bash
#
# run empty world
gz sim -v4 -r
```

### Terminal 2 (ros)

```bash
ros2 run ros_gz_bridge parameter_bridge /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock
```

### Terminal 3 (ros)

```bash
ros2 topic echo /clock
#
---
clock:
  sec: 2028
  nanosec: 785000000
---
```
