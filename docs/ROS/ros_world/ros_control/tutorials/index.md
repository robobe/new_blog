---
tags:
    - ros
    - ros2_control
    - tutorial
---

# ros2 control tutorials

Using Gazebo to simulate type of control on one joint simple robot.  
Show **yaml** control configuration and **ros2_control** tag for each type.

The ros2_control declare what the robot can do, and expose it as command and state  
The Controller YAML declare which controller to load and use by the **controller_manager**
The controller expose ROS messages to command and get states


<div class="grid-container">
    <div class="grid-item">
            <a href="position_control">
                <img src="images/position.png"  width="150" height="150"/>
                <p>position controller</p>
            </a>
        </div>
        <div class="grid-item">
             <a href="velocity_control">
             <img src="images/velocity.png"  width="150" height="150"/>
                <p>velocity controller</p>
            </a>
        </div>
    <div class="grid-item">
          <a href="effort_control">
                <img src="images/ros2_control_effort.png"  width="150" height="150">
                <p>Effort Control</p>
            </a>
    </div>
    <div class="grid-item">
          <a href="imu_broadcaster">
                <img src="images/imu.png"  width="150" height="150">
                <p>imu broadcaster</p>
            </a>
    </div>
</div>
- [ros2_control guide](https://control.ros.org/jazzy/doc/getting_started/getting_started.html)
- [ROS2 Jazzy Tutorial: ros2_control in Gazebo - Control Simulation from Scratch in Gazebo!](https://www.youtube.com/watch?v=_F8wVuiEmww)

