---
title: Mavros
tags:
    - ros
    - mavros
    - mavlink
    - ardupilot
---
{{ page_folder_links() }}

MAVLink extendable communication node for ROS2.


## Connections url

- **Serial**: /path/to/serial/device[:baudrate]
- **Serial**: serial:///path/to/serial/device[:baudrate][?ids=sysid,compid]
- **UDP**: udp://[bind_host][:port]@[remote_host[:port]][/?ids=sysid,compid]
- **TCP** client: tcp://[server_host][:port][/?ids=sysid,compid]

```bash title="ardupilot connection"
ros2 launch mavros apm.launch fcu_url:=tcp://localhost:5760 gcs_url:=udp://@localhost:14550

```

```bash title="using node.launch connect to APM SITL and control pluginlist and config"
ros2 launch mavros node.launch \
fcu_url:=tcp://0.0.0.0:5760@ \
gcs_url:=udp://@127.0.0.1:14560 \
tgt_system:=1 \
tgt_component:=1 \
pluginlists_yaml:=plugins.yaml \
config_yaml:=config.yaml
```

```bash title="using node.launch connect to APM HW and control pluginlist and config"
ros2 launch mavros node.launch \
fcu_url:=/dev/ttyACM1:115200 \
gcs_url:=udp://@127.0.0.1:14560 \
tgt_system:=1 \
tgt_component:=1 \
pluginlists_yaml:=/workspace/src/ardupilot_bringup/config/plugins.yaml \
config_yaml:=/workspace/src/ardupilot_bringup/config/config.yaml
```


!!! tip "message interval"

    ```bash
    ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 74, message_rate: 1.0}"
    ```
     
---

## Mavlink MAV_FRAME


- **GLOBAL**: Global coordinate frame with WGS84 latitude/longitude and altitude positive over mean sea level (MSL) by default
- **RELATIVE_ALT**: Altitude is relative to the vehicle **home position** rather than MSL.
- **TERRAIN_ALT**: Altitude is relative to ground level rather than MSL.
- **INT**: Latitude/longitude (in degrees) are scaled by multiplying by 1E7.
    
- **LOCAL**: Origin of local frame is fixed relative to earth.  origin of the vehicle position-estimator ("EKF").
- **BODY**: Origin of local frame travels with the vehicle. NOTE, "BODY" **does NOT** indicate alignment of frame axis with vehicle attitude.

---

## ENU and NED coordinate frame
- NED: North East Down
- ENU: East North Down (use by ROS)

### NED
This frame is widely used in aviation and maritime applications 
- North (X-axis): Points towards true North.
- East (Y-axis): Points towards East, perpendicular to the North axis.
- Down (Z-axis): Points downwards, perpendicular to the North-East plane.

### ENU
This frame is commonly used in robotics and ROS

- East (X-axis): Points towards East.
- North (Y-axis): Points towards true North, perpendicular to the East axis.
- Up (Z-axis): Points upwards, perpendicular to the East-North plane.

---

## Plugins

<div class="grid-container">
    <div class="grid-item">
        <a href="tf">
            <p>tf</p>
        </a>
    </div>
    <div class="grid-item">
        <a href="sys_status">
            <p>sys_status</p>
        </a>
    </div>
    <div class="grid-item">
        <a href="sys_time">
            <p>sys_time</p>
        </a>
    </div>
    <div class="grid-item">
        <a href="terrain">
            <p>terrain</p>
        </a>
    </div>
    <!--  -->
    <div class="grid-item">
        <a href="distance_sensor">
            <p>distance_sensor</p>
        </a>
    </div>
    <div class="grid-item">
        <a href="ahrs2">
            <p>ahrs2</p>
        </a>
    </div>
    <div class="grid-item">
        <a href="home_position">
            <p>home_position</p>
        </a>
    </div>
    <!--  -->
    <div class="grid-item">
        <a href="setpoint_velocity">
            <p>setpoint_velocity</p>
        </a>
    </div>
    <div class="grid-item">
        <a href="local_position">
            <p>local_position</p>
        </a>
    </div>
    <div class="grid-item">
        <a href="vfr_hud">
            <p>vfr_hud</p>
        </a>
    </div>
    <!--  -->
    <div class="grid-item">
        <a href="gimbal_control">
            <p>gimbal_control</p>
        </a>
    </div>
    <div class="grid-item">
        <a href="setpoint_raw">
            <p>setpoint_raw</p>
        </a>
    </div>
    <div class="grid-item">
        <a href="setpoint_attitude">
            <p>setpoint_attitude</p>
        </a>
    </div>
    <!--  -->
    <div class="grid-item">
        <a href="param">
            <p>param</p>
        </a>
    </div>
    <div class="grid-item">
        <a href="global_position">
            <p>global_position</p>
        </a>
    </div>
    <div class="grid-item">
        <a href="imu">
            <p>imu</p>
        </a>
    </div>
    <!--  -->
    <div class="grid-item">
        <a href="do_set_mode">
            <p>do_set_mode</p>
        </a>
    </div>
    <div class="grid-item">
         <a href="command">
            <p>command</p>
        </a>
    </div>
    <div class="grid-item">
        <a href="rc_io">
            <p>rc_io</p>
        </a>
    </div>
     <div class="grid-item">
        <a href="altitude">
            <p>altitude</p>
        </a>
    </div>
    <div class="grid-item">
        <a href="vision_speed_estimation">
            <p>vision_speed_estimation</p>
        </a>
    </div>
</div>

