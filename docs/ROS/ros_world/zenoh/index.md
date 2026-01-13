---
title: Zenoh networking
tags:
    - zenoh
---

Zenoh networking is a unified data-centric communication protocol
A unified data-centric communication protocol is a networking model where:

Applications communicate by sharing data, not by talking to specific endpoints — and the same protocol supports multiple communication patterns.

### Unified
A single protocol supports:

- **Publish / Subscribe** → streaming data
- **Request / Reply (Query)** → asking for data or computation
- **Storage / Replay** → retrieving past data

![alt text](images/zenoh_topolgy.png)

[TO READ](https://medium.com/@kelj/zenoh-a-protocol-that-should-be-on-your-radar-72befa697411)

<div class="grid-container">
    <div class="grid-item">
        <a href="ros_bridge">
        <img src="images/ros.png" width="150" height="150">
        <p>ROS Bridge</p>
        </a>
    </div>
    <div class="grid-item">
    <a href="python_bindings">
        <img src="images/python.png" width="150" height="150">
        <p>Python bindings</p>
        </a>
    </div>
    <div class="grid-item">
        <a href="remote_ssh">
        <img src="images/pico.png" width="150" height="150">
        <p>Zenoh pico</p>
        </a>
    </div>
     <div class="grid-item">
        <a href="rmw">
        <img src="images/zenoh_rmw.png" width="150" height="150">
        <p>ROS RMW</p>
        </a>
    </div>
    <div class="grid-item">
        <a href="cpp_bindings">
        <img src="images/cpp_bindings.png" width="150" height="150">
        <p>CPP bindings</p>
        </a>
    </div>
</div>

---

## Resource
- [Zenoh Tutorial – Part I](https://youtu.be/xNJlDIkV2uo?list=PLZDEtJusUvAa91YlGT4ugsR3_heTa1RqS)
  
---

- [zenoh json5 configuration schema](https://github.com/eclipse-zenoh/zenoh/blob/main/DEFAULT_CONFIG.json5)
- [build zenoh bridge with shm support ](ros_bridge/zenoh_bridge_build_with_shm.md)

---

## To Read
- [Zenoh](https://conferences2.sigcomm.org/acm-icn/2022/assets/zenoh-4-Zenoh-and-Zenoh-Flow-Hands-on-e8cbd760e0b88b74417fb1c14d1d373b5ce2a094bc29b5f1a0bfd8e52030c151.pdf)
- [Zenoh ROSCon 2024 Workshop](https://github.com/ZettaScaleLabs/roscon2024_workshop/tree/main)
  