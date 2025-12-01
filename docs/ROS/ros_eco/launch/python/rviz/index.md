---
title: Launch RVIZ and robot_description
tags:
    - ros
    - rviz
    - launch
    - xacro
    - robot_description
---


{{ page_folder_links() }}

Launch file template to load robot_description from xacro file and view in RVIZ

- Set rviz config file path using `PathJoinSubstitution`
- Set xacro file path using `PathJoinSubstitution`
- Use `command` to run **xacro** 
- Use `joint_state_publisher_gui` to control joints



  

```python title="description.launch.py"
--8<-- "docs/ROS/ros_eco/launch/rviz/code/description.launch.py"
```

## Install

```bash title="install"
sudo apt install ros-jazzy-joint-state-publisher-gui
```