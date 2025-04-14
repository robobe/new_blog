---
tags:
    - vsc
    - ros
    - tools
---

# VSC Tool

Command-line tools for maintaining a workspace of projects from multiple version-control systems. 
[ROS Wiki](http://wiki.ros.org/vcstool)


```bash title="install"
sudo apt install python3-vcstool
```

## Repos file

```yaml title="project.repo"
repositories:
  my_robot_pkg:
    type: git
    url: https://github.com/my-org/my_robot_pkg.git
    version: main
  nav2_bringup:
    type: git
    url: https://github.com/ros-planning/navigation2.git
    version: humble
```

## Import and update

```bash title="import"
vsc import < project.repos
```

```bash title="update"
vsc pull
```

## Export

```bash title="export"
vsc export > project.repos
```

## Use case

- Import `.repos` file from a GitHub repository or using other whays like `curl` or `wget`.
- Run `vsc import` to clone the repositories.