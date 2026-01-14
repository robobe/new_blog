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


### Demo: Minimal pub/sub using python binding
minimal Pub/Sub example the zenoh default config, 

```python
import zenoh
import time
import multiprocessing

zenoh.init_log_from_env_or("DEBUG")

KEY_EXPRESSION = "demo/example"

def publisher():
    config = zenoh.Config()
    session = zenoh.open(config)
    pub = session.declare_publisher(KEY_EXPRESSION)
    while True:
        pub.put('Hello, Zenoh!')
        time.sleep(1)

def subscriber():
    config = zenoh.Config()
    session = zenoh.open(config)
    sub = session.declare_subscriber(KEY_EXPRESSION, lambda sample: print(f'Received: {sample.payload.to_string()}'))
    while True:
        time.sleep(1)

if __name__ == '__main__':
    pub_process = multiprocessing.Process(target=publisher)
    sub_process = multiprocessing.Process(target=subscriber)

    pub_process.start()
    sub_process.start()

    pub_process.join()
    sub_process.join()
```

```bash output
zenoh::net::runtime: Using ZID: 5c878ff12d11c0d9330259f34cd29fac
zenoh::net::runtime: Using ZID: 42209fd2b31a654f83739cebf2e2b000
zenoh::net::runtime::orchestrator: Zenoh can be reached at: tcp/[fe80::b022:f1eb:e86:2654]:37185
zenoh::net::runtime::orchestrator: Zenoh can be reached at: tcp/[fe80::b022:f1eb:e86:2654]:45353
zenoh::net::runtime::orchestrator: Zenoh can be reached at: tcp/10.100.102.15:37185
zenoh::net::runtime::orchestrator: Zenoh can be reached at: tcp/10.100.102.15:45353
zenoh::net::runtime::orchestrator: zenohd listening scout messages on 224.0.0.224:7446
zenoh::net::runtime::orchestrator: zenohd listening scout messages on 224.0.0.224:7446
```

-  each of the zenoh process listen to multicast on ``
-  the transport between the peer node base on tcp
-  

---

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
- [TO READ](https://medium.com/@kelj/zenoh-a-protocol-that-should-be-on-your-radar-72befa697411)
  
---

- [zenoh json5 configuration schema](https://github.com/eclipse-zenoh/zenoh/blob/main/DEFAULT_CONFIG.json5)
- [build zenoh bridge with shm support ](ros_bridge/zenoh_bridge_build_with_shm.md)

---

## To Read
- [Zenoh](https://conferences2.sigcomm.org/acm-icn/2022/assets/zenoh-4-Zenoh-and-Zenoh-Flow-Hands-on-e8cbd760e0b88b74417fb1c14d1d373b5ce2a094bc29b5f1a0bfd8e52030c151.pdf)
- [Zenoh ROSCon 2024 Workshop](https://github.com/ZettaScaleLabs/roscon2024_workshop/tree/main)
  