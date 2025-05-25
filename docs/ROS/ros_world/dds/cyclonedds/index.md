---
tags:
    - cyclonedds
    - dds
---


# Cyclone DDS
[CycloneDDS web site](https://cyclonedds.io/)

## install

```
sudo apt install ros-humble-rmw-cyclonedds-cpp
```


### CycloneDDS XML configuration

```bash
export CYCLONEDDS_URI=file://$PWD/cyclonedds.xml
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

!!! tip "cyclonedds version"
    
    CycloneDDS has xml configuration file for runtime configuration that changed between versions.

    ```bash
    dpkg -s ros-humble-cyclonedds
    ```

    [ros humble ver(0.10.5)](https://github.com/eclipse-cyclonedds/cyclonedds/tree/releases/0.10.x#run-time-configuration)
     
---

<div class="grid-container">
    <div class="grid-item">
            <a href="tutorials">
                <img src=""  width="150" height="150"/>
                <p>configuration</p>
            </a>
        </div>
        <div class="grid-item">
             <a href="cyclonedds_tools">
                <img src=""  width="150" height="150">
                <p>tools</p>
            </a>
        </div>
    <div class="grid-item">
          <a href="cyclonedds_tips">
                <img src=""  width="150" height="150">
                <p>tips / best practice</p>
            </a>
    </div>
</div>

## CycloneDDS protocols

SSPDP and SEDP are the two core discovery protocols that enable nodes to find and communicate with each other automatically.

| Protocol | Full Name                             | Role                                                                                      |
| -------- | ------------------------------------- | ----------------------------------------------------------------------------------------- |
| **SPDP** | Simple Participant Discovery Protocol | Discovers **other DDS participants (nodes)** on the network                               |
| **SEDP** | Simple Endpoint Discovery Protocol    | Discovers the **publishers and subscribers** (topics, types, QoS) within each participant |


**SPDP (Simple Participant Discovery Protocol)**
is the first step in DDS discovery. It:

    - Broadcasts participant info (GUID, capabilities) using multicast on UDP port 7400 (domain 0).
    - Helps nodes find each other without a centralized registry.
    - Uses periodic announcements.
    - In CycloneDDS:
        - SPDP is what shows up in tcpdump as packets to 239.255.0.1:7400 (default multicast group).

**SEDP (Simple Endpoint Discovery Protocol):**

Once SPDP discovers participants, SEDP kicks in to share:

    - What topics each node publishes or subscribes to
    - Message types
    - QoS policies



---

## Demo: CycloneDDS between two machines
Ros humble cyclonedds version 0.10.5

### machine 1

```xml title="cyclonedds.xml"
<?xml version="1.0" encoding="UTF-8"?>
<CycloneDDS xmlns="https://cdds.io/config" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
  xsi:schemaLocation="https://cdds.io/config https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd">
  <Domain Id="any">
    <General>
       <AllowMulticast>true</AllowMulticast>
       <!-- <AllowMulticast>spdp</AllowMulticast> -->
    </General>
  </Domain>
</CycloneDDS>

```

!!! note "AllowMulticast"
    Both configurations are valid `true` and `spdp` .

    The <AllowMulticast> setting controls which parts of DDS discovery and communication are allowed to use multicast.

    - `true`: allow all multicast traffic,
    - `spdp`: only discovery uses multicast, other communication is unicast
    - `false`: unicast only
     

### machine 2

```xml title="cyclonedds.xml"
<?xml version="1.0" encoding="UTF-8"?>
<CycloneDDS xmlns="https://cdds.io/config" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
  xsi:schemaLocation="https://cdds.io/config https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd">
  <Domain Id="any">
    <General>
      <!-- <AllowMulticast>true</AllowMulticast> -->
        <Interfaces>
          <NetworkInterface autodetermine="false" name="enx5c857e356893"/>
        </Interfaces>
    </General>
  
  </Domain>

```
!!! note "NetworkInterface"
    The `NetworkInterface` setting is used to specify which network interface to use for communication. 

    - `autodetermine="false"`: disables automatic detection of the network interface.
    - `name="enx5c857e356893"`: specifies the name of the network interface to use.

    
     


### Test

```bash
# machine 1
ros2 topic pub xxx std_msgs/String "data: hello world" -r 1
```

```bash
# machine 2
ros2 topic echo xxx
```

---

## System configuration recommendation
[ros2 rmw cyclone settings](https://github.com/ros2/rmw_cyclonedds?tab=readme-ov-file#performance-recommendations)

- net.core.rmem_max
- net.core.rmem_default

```bash
# Add settings to sysctl 
echo "net.core.rmem_max=8388608\nnet.core.rmem_default=8388608\n" | sudo tee /etc/sysctl.d/60-cyclonedds.conf
```



---

## Posts

- [cyclonedds_multicast](cyclonedds_multicast.md)
- [cyclone shm](cyclinedds_shm.md)