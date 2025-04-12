---
tags:
    - gazebo
    - harmonic
    - gz
    - simulation
    - ros
    - bridge
---

# Gazebo harmonic bridge
Connect Gazebo and ROS2 using the bridge


Install gazebo harmonic on docker image [more](Simulation/Gazebo/gz_harmonic_docker/)

<details>
    <summary>humble bridge</summary>

     
Install bridge from osrf repository

```bash title="set gazebo and bridge in dockerfile"
RUN curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
  && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null \
  && apt-get update && apt-get install -q -y --no-install-recommends \
  gz-harmonic \
  ros-humble-ros-gzharmonic \
  && rm -rf /var/lib/apt/lists/*
```

```bash
apt-get install ros-humble-ros-gzharmonic
```
</details>



### Install
camera_info
```

Bridge send data from gz to ros one way


     


```bash title="bridge"
ros2 run ros_gz_bridge parameter_bridge /vehicle/camera@sensor_msgs/msg/Image[gz.msgs.Image

```

