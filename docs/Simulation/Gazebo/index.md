---
tags:
    - gazebo
    - gz
    - harmonic
---

# Gazebo harmonic

<div class="grid-container">
    <div class="grid-item">
        <a href="vscode">
        <img src="images/vscode.png" width="150" height="150">
        <p>VSCode dev</p>
        </a>
    </div>
    <div class="grid-item">
    <a href="sdf_format">
        <img src="images/sdf_format.png" width="300" height="300">
        <p>SDF Format</p>
        </a>
    </div>
    <div class="grid-item">
        <a href="custom_plugins">
        <img src="images/custom_plugin.png" width="150" height="150">
        <p>Custom plugins</p>
        </a>
    </div>
    <div class="grid-item">
        <a href="bindings">
        <p>Bindings</p>
        </a>
    </div>
    <div class="grid-item">
        <a href="/ROS/ros_eco/urdf_xacro_gz_plugin/gazebo_harmonic/">
        <p>ROS Bridge</p>
        </a>
    </div>
    <div class="grid-item">
        <a href="demo_worlds">
        <p>Demo worlds</p>
        </a>
    </div>
</div>


## Plugins

<div class="grid-container">
    <div class="grid-item">
        <a href="sensors">
            <p>Sensors</p>
        </a>
    </div>
    <div class="grid-item">
        <a href="plugins">
            <p>plugins</p>
        </a>
    </div>
    <div class="grid-item">
        <a href="gui_plugins">
            <p>gui-plugins</p>
        </a>
    </div>
</div>

## Physics

<div class="grid-container">
    <div class="grid-item">
        <a href="sdf_format/surface_friction/">
            <p>surface friction</p>
        </a>
    </div>
    <div class="grid-item">
        <a href="sdf_format/joint_friction/">
            <p>joint friction</p>
        </a>
    </div>
</div>

## Running with nvidia
My laptop has two graphics card and it config to `on-demand` using **Prime profiles**

for running gz sim with nvidia card switch to nvidia mode using `sudo prime-select nvidia` or using 

```bash
__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia gz sim
```

!!! tip "check"
    using `nvidia-smi` to check that gz sim use nvidia card


!!! tip "prime-select`
    prime-select is an Ubuntu-specific utility used to manage hybrid graphics on laptops

    ```bash
    # check mode
    prime-select query
    ```
    

```bash title="create alias"
alias gznvidia='__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia gz sim'
```

---

## Environment Variables

!!! tip "migration from ign"
    IGNITION_GAZEBO -> GZ_SIM
     

| Name  | Description  |
|---|---|
| **SIM**  |   |
| GZ_SIM_RESOURCE_PATH  | where to search for `world`,`models` include meshes, textures, materials  |
| GZ_SIM_SYSTEM_PLUGIN_PATH  | where to find compiled system plugins (.so files) |
| **TRANSPORT**  |   |
| GZ_TRANSPORT_IP  |   |
| GZ_DISCOVERY_SERVER  |   |
| GZ_PARTITION  |   |

