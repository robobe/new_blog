---
title: FastDDS for ROS2 
tags:
    - fastdds
    - dds
---

<div class="grid-container">
    <div class="grid-item">
        <a href="dds">
            <p>DDS</p>
        </a>
    </div>
    <div class="grid-item">
        <a href="#environment-variables">
            <p>Environment variables</p>
        </a>
    </div>
    <div class="grid-item">
        <a href="xml_config">
            <p>XML control</p>
        </a>
    </div>
</div>


## Environment Variables

Using environment variables that actually control how DDS behaves.

### Select DDS implementation
```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

### Domain id

```bash
export ROS_DOMAIN_ID=0
```

!!! tip ""
    Node must have the same **domain id** to communicate

### Transport control

- DEFAULT → UDP + SHM (auto)
- UDPv4 → only UDP
- SHM → shared memory only

```bash
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
```

### XML profile configuration

```bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/path/to/fastdds.xml
```