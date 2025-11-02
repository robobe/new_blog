---
title: PyBullet Simulator
tags:
    - pybullet
    - simulation
    - bullet
---

{{ page_folder_links() }}

Physics simulation for games, visual effects, robotics and reinforcement learning.

<div class="grid-container">
     <div class="grid-item">
            <a href="dev_env">
            <img src="images/vscode.png"  width="150" height="150">
            <p>VSCode dev env</p></a>
        </div>
</div>

## Install
Dowload git

```bash
#run
pip install -e .
```

## Demo

```python title="hello.py"
import pybullet as p
import pybullet_data
import time

# Connect to PyBullet with GUI
p.connect(p.GUI, options="--opengl2 --egl")

# Set search path to PyBullet’s default data
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Load a simple plane and a cube
plane_id = p.loadURDF("plane.urdf")
cube_id = p.loadURDF("r2d2.urdf", [0, 0, 1])

# Add gravity
p.setGravity(0, 0, -9.8)

# Run simulation loop
while True:
    p.stepSimulation()
    time.sleep(1./240.)


```

```bash title="force opengl run on nvidia"
__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia python3 hello.py
```

---

### Demos and ...

[PyBullet Quickstart Guide](https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit?tab=t.0#heading=h.btdfuxtf2f72)

<div class="grid-container">
     <div class="grid-item">
            <a href="tutorials/change_ground_texture.md">
            <p>Ground Texture</p></a>
        </div>
    <div class="grid-item">
       <a href="tutorials/camera.md">
            <p>Camera</p></a>
    </div>
    <div class="grid-item">
        <a href="dev_env">
            <p>VSCode dev environment</p>
        </a>
    </div>
</div>


---

- [PyBullet](https://simulately.wiki/docs/category/pybullet)
- https://medium.com/@reflectrobotics/scara-robot-modeling-and-simulation-with-pybullet-7bb204958763
- [ROS-PyBullet Interface](https://ros-pybullet.github.io/ros_pybullet_interface/)
- [ akinami3PybulletRobotics ](https://github.com/akinami3/PybulletRobotics)
- [PyBullet and Control Algorithms Workshop - La Robo Liga Event ](https://www.youtube.com/watch?v=RkHvUSGgw6Q)
- [Quick start guide](https://raw.githubusercontent.com/bulletphysics/bullet3/master/docs/pybullet_quickstartguide.pdf)
- [PyBullet Webinar 1](https://github.com/reflectrobotics/webinar/blob/master/robot.py)
- [pybullet-imu-viz](https://github.com/robofoundry/pybullet-imu-viz/tree/main)
- [intor](https://github.com/assadollahi/pyBulletIntro)
- [How to Control a Robot with Python](https://towardsdatascience.com/how-to-control-a-robot-with-python/)
- [ PyBullet workshop for NTU students : Day-1 ](https://youtu.be/KaiznOkKkdA)
- [ PyBullet workshop for NTU students : Day-2 ](https://youtu.be/9dTQoyXaIDI)
- [ PyBullet workshop for NTU students : Day-3 ](https://youtu.be/-DLLnG_XEiw)
---

## Projects

- [balance bot RL](https://github.com/yconst/balance-bot/tree/master)
- [Gym-Line-Follower](https://github.com/nplan/gym-line-follower)