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
     