---
tags:
    - xacro
    - urdf
---

# URDF, XACRO and Gazebo

<div class="grid-container">
     <div class="grid-item">
            <a href="urdf">
            <img src="images/urdf.png"  width="150" height="150">
            <p>URDF</p></a>
        </div>
    <div class="grid-item">
       <a href="xacro">
            <img src="images/urdf.png"  width="150" height="150">
            <p>xacro</p></a>
    </div>
    <div class="grid-item">
        <a href="gazebo_classic">
            <img src="images/gazebo_classic.png"  width="150" height="150">
            <p>Gazebo classic</p></a>
    </div>
     <div class="grid-item">
        <a href="gazebo_harmonic">
            <img src="images/gazebo_harmonic.png"  width="150" height="150">
            <p>Gazebo harmonic</p></a>
    </div>
   </div>

## URDF

!!! tip "Quick run rviz"

    Quick visualize and manipulate URDF file using RVIZ and joint state publisher
    ```
    sudo apt install ros-humble-urdf-tutorial

    ```

    ```
    ros2 launch urdf_tutorial display.launch.py model:=/gz_tutorial_description/urdf/my_robot.urdf
    ```

    ### more arguments
    - **rvizconfig**: absolute path to rviz config
     



---

## ToRead
- [QoS mismatch between gazebo_ros_camera and gazebo_ros_video](https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1218)