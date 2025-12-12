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



!!! tip
    install pyi for intellisense
    [pybulley.pyi download](code/pybullet.pyi)


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

- pybullet_data provide many URDF examples.
- `loadURDF` return integer that use to refer the robot in other pybullet commands/


## Demo: Get Position and Orientation

```python title="Position orientation and apply external force"
import pybullet as p
import pybullet_data
import time

# Connect to PyBullet with GUI
p.connect(p.GUI, options="--opengl2 --egl")

# Set search path to PyBullet’s default data
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Load a simple plane and a cube
plane_id = p.loadURDF("plane.urdf")
carId = p.loadURDF(“racecar/racecar.urdf”, basePosition=[0,0,0.2])
position, orientation = p.getBasePositionAndOrientation(carId)
for _ in range(100): 
    p.stepSimulation()
```

---

### Demos and ...

[PyBullet Quickstart Guide](https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit?tab=t.0#heading=h.btdfuxtf2f72)

<div class="grid-container">
 <div class="grid-item">
            <a href="tutorials/loadurdf">
            <p>loadURDF</p></a>
        </div>
     <div class="grid-item">
            <a href="tutorials/links_and_joints">
            <p>Links and Joints</p></a>
        </div>
    <div class="grid-item">
       <a href="tutorials/camera">
            <p>Camera</p></a>
    </div>
    <div class="grid-item">
        <a href="dev_env">
            <p>VSCode dev environment</p>
        </a>
    </div>
    <div class="grid-item">
        <a href="gui_control">
            <p>GUI and TK inter</p>
        </a>
    </div>

</div>


---
- [online urdf](http://mymodelrobot.appspot.com/)
- [PyBullet](https://simulately.wiki/docs/category/pybullet)
- https://medium.com/@reflectrobotics/scara-robot-modeling-and-simulation-with-pybullet-7bb204958763
- [ROS-PyBullet Interface](https://ros-pybullet.github.io/ros_pybullet_interface/)
- [ akinami3PybulletRobotics ](https://github.com/akinami3/PybulletRobotics)
- [PyBullet and Control Algorithms Workshop - La Robo Liga Event ](https://www.youtube.com/watch?v=RkHvUSGgw6Q)
- [Quick start guide](https://raw.githubusercontent.com/bulletphysics/bullet3/master/docs/pybullet_quickstartguide.pdf)
- [PyBullet Webinar 1](https://github.com/reflectrobotics/webinar/blob/master/robot.py)

- [intor](https://github.com/assadollahi/pyBulletIntro)
- [How to Control a Robot with Python](https://towardsdatascience.com/how-to-control-a-robot-with-python/)
- [ PyBullet workshop for NTU students : Day-1 ](https://youtu.be/KaiznOkKkdA)
- [ PyBullet workshop for NTU students : Day-2 ](https://youtu.be/9dTQoyXaIDI)
- [ PyBullet workshop for NTU students : Day-3 ](https://youtu.be/-DLLnG_XEiw)
- [Creating OpenAI Gym Environments with PyBullet (Part 1)](https://gerardmaggiolino.medium.com/creating-openai-gym-environments-with-pybullet-part-1-13895a622b24)
- [Creating OpenAI Gym Environments with PyBullet (Part 2)](https://gerardmaggiolino.medium.com/creating-openai-gym-environments-with-pybullet-part-2-a1441b9a4d8e)
---

## Projects

- [balance bot RL](https://github.com/yconst/balance-bot/tree/master)
- [Gym-Line-Follower](https://github.com/nplan/gym-line-follower)
- [pybullet cartpole](https://gitlab.cs.washington.edu/ym2552/bullet3/-/blob/6e4707df5fa1f9927109e89a7cd2a6d6a6ddd072/examples/pybullet/gym/pybullet_envs/bullet/cartpole_bullet.py)