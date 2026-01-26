---
title: Use gazebo harmonic with vscode devcontainer
tags:
    - gazebo
    - gz
    - harmonic
    - docker
    - vscode
    - devcontainer
---





## Use VSCode and docker
- Using docker-compose
- Using devcontainer to run the docker compose

```
├── .devcontainer
│   ├── Dockerfile
│   └── devcontainer.json
├── docker-compose.yaml
├── .gitignore
└── worlds
        └── empty.world

```

<details>
<summary>Dockerfile</summary>
```
--8<-- "docs/Simulation/Gazebo/vscode/code/Dockerfile"
```
</details>

<details>
<summary>docker-compose</summary>
```yaml title="docker-compose.yaml"
--8<-- "docs/Simulation/Gazebo/vscode/code/docker-compose.yaml"
```
</details>

!!! tip "nvidia glx"
    - __NV_PRIME_RENDER_OFFLOAD=1
    - __GLX_VENDOR_LIBRARY_NAME=nvidia
    

```yaml title=".devcontainer/devcontainer.json"
--8<-- "docs/Simulation/Gazebo/vscode/code/devcontainer.json"
```

---

## Run the simulation

<details>
    <summary>minimal world</summary>

```xml
--8<-- "docs/Simulation/Gazebo/vscode/code/minimal.world"
```
</details>


### Environment helper file
- Change prompt
- Set Environment variables
- Add keyboard shortcuts


```bash title="env.sh"

export GZ_SIM_RESOURCE_PATH=$(pwd)/models:$(pwd)/worlds:$GZ_SIM_RESOURCE_PATH
export GZ_SIM_SYSTEM_PLUGIN_PATH=$(pwd)/bin:$GZ_SIM_SYSTEM_PLUGIN_PATH
echo '🤖 Environment ready!'
# bash key bindings
# replace bringup with full bringup name
bind '"\C-b": "gz sim -v 4 -r mini.world"'

# Function to get git branch
parse_git_branch() {
    git branch 2> /dev/null | sed -e '/^[^*]/d' -e 's/* \(.*\)/[\1]/'
}

# Custom PS1 with turtle icon and git branch
export PS1="🤖 \[\033[32m\]\u@\h\[\033[00m\]:\[\033[34m\]\w\[\033[33m\]\$(parse_git_branch)\[\033[00m\]\$ "
```

### Usage

```bash
# run the simulation
gz sim -r v 4 gz_tutorial/worlds/mini.world 
```

![alt text](images/minimal_world.png)


```bash title="gz topic"
gz topic --list
gz topic -e -t /clock
#

sim {
  sec: 326
  nsec: 476000000
}
```

---

## Gz Transport
gz_transport is Gazebo's communication middleware, used for inter-process communication (IPC) between different Gazebo components — like sensors, plugins, UI, and even ROS bridges.


- **GZ_PARTITION**:	Isolates topic namespaces between different simulations
- **GZ_DISCOVERY_SERVER**:	IP address of the main discovery server (usually the Gazebo server machine)
- **GZ_TRANSPORT_IP**:	IP address the local process uses to advertise itself to others

---

## Resource
- [Gazebo Simulator : 5 Ways to Speedup Simulations](https://www.blackcoffeerobotics.com/blog/gazebo-simulator-5-ways-to-speedup-simulations)