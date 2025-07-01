---
tags:
    - ros
    - gazebo
    - harmonic
    - bridge
    - jazzy
---

# ROS2 Gazebo harmonic integration

<div class="grid-container">
     <div class="grid-item">
            <a href="jazzy_bridge">
            <img src="images/bridge.png"  width="150" height="150">
            <p>Bridge</p></a>
        </div>
    <div class="grid-item">
       <a href="jazzy_interface">
            <img src="images/ros_gz_interface.png"  width="150" height="150">
            <p>Interface</p></a>
    </div>
    <div class="grid-item">
        <a href="project_template">
            <img src="images/project_template.png"  width="150" height="150">
            <p>Project</p></a>
    </div>
   
   </div>


## Bridge clock from gazebo

```bash title="/clock topic"
ros2 run ros_gz_bridge parameter_bridge /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock
```

