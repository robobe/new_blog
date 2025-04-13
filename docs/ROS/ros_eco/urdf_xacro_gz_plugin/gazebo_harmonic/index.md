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
            <img src="images/urdf.png"  width="150" height="150">
            <p>Bridge</p></a>
        </div>
    <div class="grid-item">
       <a href="jazzy_interface">
            <img src="images/urdf.png"  width="150" height="150">
            <p>Interface</p></a>
    </div>
    <div class="grid-item">
        <a href="jazzy_launch">
            <img src="images/gazebo_classic.png"  width="150" height="150">
            <p>Launch</p></a>
    </div>
   
   </div>

## Bridge
The bridge allows you to connect ROS 2 topics with Gazebo Harmonic topics so you can:

- Control simulation objects from ROS 2 (e.g., publish velocity commands).
- Get data from simulation (e.g., sensor readings, /clock).
- Sync **time** between ROS 2 and Gazebo.

!!! tip parameter_bridge
    ```
    parameter_bridge <topic@ROS2_type@gz_type>
    ```

!!! tip sign between ros2 type to gz type
    - **@** : a bidirectional bridge, 
    - **[** : a bridge from Gazebo to ROS,
    - **]** : a bridge from ROS to Gazebo.


### bridge yaml config
[github](https://github.com/gazebosim/ros_gz/tree/jazzy/ros_gz_bridge#example-5-configuring-the-bridge-via-yaml)


```bash
ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=$WORKSPACE/test/config/full.yaml
```

## Demo
Simulate using bridge, send message from side to side
(no need to run gz sim for this check)

send string message from gz to ros and from ros to gz using cli

- Run bridge
- Pub from GZ echo in ROS
- Pub from ROS echo in GZ


```bash title="Terminal1: bridge"
ros2 run ros_gz_bridge parameter_bridge /chatter@std_msgs/msg/String@gz.msgs.StringMsg
#
[INFO] [1734974012.615668960] [ros_gz_bridge]: Creating GZ->ROS Bridge: [/chatter (gz.msgs.StringMsg) -> /chatter (std_msgs/msg/String)] (Lazy 0)
[INFO] [1734974012.616345817] [ros_gz_bridge]: Creating ROS->GZ Bridge: [/chatter (std_msgs/msg/String) -> /chatter (gz.msgs.StringMsg)] (Lazy 0)
```

#### pub data from gz to ros
```bash title="Terminal2: ros subscribe"
ros2 topic echo /chatter
```

```bash title="Terminal3: gz pub"
gz topic -t /chatter -m gz.msgs.StringMsg -p 'data:"Hello"'
```

#### pub data from ros to gz

```bash title="Terminal2: ros pub"
ros2 topic pub /chatter std_msgs/msg/String "data: 'Hi'"
```

```bash title="Terminal3: gz sub"
gz topic -e -t /chatter
```

---

### Resources
- [Sensors services and interfaces]()
- [ROS + Gazebo Sim demos](https://github.com/gazebosim/ros_gz/tree/jazzy/ros_gz_sim_demos)
