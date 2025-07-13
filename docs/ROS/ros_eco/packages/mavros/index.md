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
        <a href="sys_state">
            <p>sys_state</p>
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
</div>

