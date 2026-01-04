---
title: Using GPIO with ros2_control
tags:
  - ros2
  - ros_control
  - raspberry pi
  - rpi
---

## LAB
- RPI 5
- ubuntu 24.04
- ROS Jazzy
- VSCode remote

---

### gpiod

```
sudo apt install gpiod
```

!!! warning "gpiod api changed"
    From version 2.x the api changed
    

!!! tip "my rpi prompt"
    ```
    if [ -n "$SSH_CONNECTION" ]; then
        if [ "$EUID" -eq 0 ]; then
            PS1='🍓\[\e[1;41m\][ROOT SSH \u@\h]\[\e[0m\] \w # '
        else
            PS1='🍓\[\e[1;31m\][SSH \u@\h]\[\e[0m\] \w \$ '
        fi
    fi
    ```
    
#### gpiodetect

#### gpioinfo

```bash
# v2
gpioinfo -c gpiochip0
#v1
gpioinfo gpiochip4
```

#### gpioset/ gpioget
```bash
#v2
gpioset -c gpiochip0 GPIO17=1
gpioset -c gpiochip0 17=1
#v1
# offset only
gpioset gpiochip4 17=1

#v2
gpioget -c gpiochip0 GPIO17
gpioget -c gpiochip0 17
#v1
gpioget gpiochip4 17
```

### ROS2

[official install doc](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)

---

### ROS2_control

```
sudo apt install ros-jazzy-ros2-control
sudo apt install ros-jazzy-ros2-controllers
```
