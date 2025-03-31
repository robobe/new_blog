---
tags:
    - nvidia
    - isaac
    - simulation
---

# NVidia isaac

- Isaac SDK: A development kit for creating AI-based robotics applications.

- Isaac ROS: A set of ROS 2-compatible packages optimized for NVIDIA hardware (Jetson).

- Isaac Sim: A simulation tool built on Omniverse for training and testing robotics algorithms in a virtual environment.

!!! note "Isaac vs Isaac Lab"
    **NVIDIA Isaac** is a broad robotics software platform designed for AI-powered autonomous robots.
    **Isaac Lab** is an **extension** of Isaac Sim, specifically focused on reinforcement learning (RL) and AI training for robotics.
     
## Isaac Automator
[Running Isaac Lab in the Cloud](https://isaac-sim.github.io/IsaacLab/main/source/setup/installation/cloud_installation.html)
```
git clone https://github.com/isaac-sim/IsaacAutomator.git
```

### NGC API key

Generate [NGC API Key](https://docs.nvidia.com/ngc/gpu-cloud/ngc-user-guide/index.html#generating-api-key)


### login

```
docker login nvcr.io
```

```
Username: $oauthtoken
Password: <Your NGC API Key>
```

### Run

```
./build
```

```
./run
```

```
./deploy-gcp
```

!!! tip "ngc api"
    paste the key without prefix
     

<details>
    <summary>end of installation</summary>

******************************************
* Isaac Sim is deployed at 34.134.62.169 *
******************************************

* To connect to Isaac Sim via SSH:

ssh -i state/dense-onion/key.pem -o StrictHostKeyChecking=no ubuntu@34.134.62.169

* To connect to Isaac Sim via noVNC:

1. Open http://34.134.62.169:6080/vnc.html?host=34.134.62.169&port=6080 in your browser.
2. Click "Connect" and use password "msUc4xpLXb"

* To connect to Isaac Sim via NoMachine:

0. Download NoMachine client at https://downloads.nomachine.com/, install and launch it.
1. Click "Add" button.
2. Enter Host: "34.134.62.169".
3. In "Configuration" > "Use key-based authentication with a key you provide",
   select file "state/dense-onion/key.pem".
4. Click "Connect" button.
5. Enter "ubuntu" as a username when prompted.


!!! note "key.pem"
     The key exists in automator docker `/state` folder
</details>

