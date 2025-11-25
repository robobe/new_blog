---
tags:
    - ros
    - jazzy
    - harmonic
    - setup
    - vscode
    - docker
---

# Jazzy Harmonic Bridge Setup

## Setup
- jazzy docker image
- harmonic docker image
- both run using docker-compose

## gz_transport
gz_transport is Gazebo's communication middleware, used for inter-process communication (IPC) between different Gazebo components — like sensors, plugins, UI, and even ROS bridges.

- **GZ_PARTITION**:	Isolates topic namespaces between different simulations
- **GZ_DISCOVERY_SERVER**:	IP address of the main discovery server (usually the Gazebo server machine)
- **GZ_TRANSPORT_IP**:	IP address the local process uses to advertise itself to others

### jazzy

- [docker-compose.yaml]()

#### bridge configuration

<details>
    <summary>config/gz_bridge.yaml</summary>

```yaml
--8<-- "docs/ROS/dev_environment/tutorials/jazzy_harmonic_setup/code/gz_bridge.yaml"
```
</details>


<details>
    <summary>launch/clock.launch.py</summary>

```python
--8<-- "docs/ROS/dev_environment/tutorials/jazzy_harmonic_setup/code/clock.launch.py"
```
</details>


### harmonic

- [Dockerfile]()
- [docker-compose.yaml]()