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

## Plugins

<div class="grid-container">
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
</div>

