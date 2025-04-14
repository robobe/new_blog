---
tags:
    - ros
    - build system
    - colcon
    - ament
---
# Build System

<div class="grid-container">
    <div class="grid-item">
        <a href="tips_settings">
        <img src="images/colcon.png" width="150" height="150">
        <p>Colcon</p>
        </a>
    </div>
    <div class="grid-item">
    <a href="ament">
        <img src="images/ament.png" width="150" height="150">
        <p>ament</p>
        </a>
    </div>
    <div class="grid-item">
        <p>TBD</p>
    </div>
    
</div>

[ROS: The build system](https://docs.ros.org/en/jazzy/Concepts/Advanced/About-Build-System.html)

## colcon-clean
Extension for colcon to clean package workspaces

```bash
sudo apt install python3-colcon-clean
```

```bash title="Usage"
colcon clean workspace

colcon clean packages 
```

---

## colcon-mixin
Colcon mixin is a powerful feature of the colcon build system that allows you to define and reuse commonly used build configurations. It's particularly useful for cross-compilation scenarios because it helps you avoid typing long, complex build commands repeatedly. [cross compiler example](/ROS/dev_environment/build/demo/build_deploy_ros_project_4/)

```bash
sudo apt install python3-colcon-mixin
```

```bash title="Usage"
colcon mixin list
```

