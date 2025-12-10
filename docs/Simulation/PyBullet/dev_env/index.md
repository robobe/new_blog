---
title: PyBullet VSCode dev environment
tags:
    - pybullet
    - vscode
    - dev
---


{{ page_folder_links() }}

```
.
├── .devcontainer
│   ├── devcontainer.json
│   └── Dockerfile
├── docker-compose.yml
├── README.md
├── requirements.txt
└── .vscode
    └── settings.json
```

<details>
    <summary>.devcontainer/devcontainer.json</summary>

```json
--8<-- "docs/Simulation/PyBullet/dev_env/code/.devcontainer/devcontainer.json"
```
</details>


<details>
    <summary>.devcontainer/Dockerfile</summary>

```dockerfile
--8<-- "docs/Simulation/PyBullet/dev_env/code/.devcontainer/Dockerfile"
```
</details>


<details>
    <summary>docker-compose.yml</summary>

```yaml
--8<-- "docs/Simulation/PyBullet/dev_env/code/docker-compose.yml"
```
</details>

<details>
    <summary>requirements.txt</summary>

```txt
--8<-- "docs/Simulation/PyBullet/dev_env/code/requirements.txt"
```
</details>


## pyi

[pybulley.pyi download](code/pybullet.pyi)
copy the `pyi` file to library location

```bash
>>> import pybullet
pybullet build time: Dec  8 2025 19:55:19
>>> pybullet.__file__
'/usr/local/lib/python3.12/dist-packages/pybullet.cpython-312-x86_64-linux-gnu.so'
```

---

## Check OpenGL backend

```bash
glxinfo | grep -i vendor
#
server glx vendor string: SGI
client glx vendor string: NVIDIA Corporation
OpenGL vendor string: NVIDIA Corporation



glxinfo | grep -i renderer
#
OpenGL renderer string: NVIDIA GeForce MX450/PCIe/SSE2
```



```py title="hello_bullet.py"
import pybullet as p
import time
import pybullet_data

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
# Add gravity
p.setGravity(0, 0, -9.8)
plane_id = p.loadURDF("plane.urdf")
# cube_id = p.loadURDF("urdf/self_balance.urdf", [0, 0, 1])
robot = p.loadURDF("kuka_iiwa/model.urdf", useFixedBase=True)

# Run simulation loop
while True:
    p.stepSimulation()
    time.sleep(1./240.)
```

```bash
python3 hello_bullet.py
#
pybullet build time: Nov 18 2025 13:39:23
startThreads creating 1 threads.
starting thread 0
started thread 0 
argc=2
argv[0] = --unused
argv[1] = --start_demo_name=Physics Server
ExampleBrowserThreadFunc started
X11 functions dynamically loaded using dlopen/dlsym OK!
X11 functions dynamically loaded using dlopen/dlsym OK!
Creating context
Created GL 3.3 context
Direct GLX rendering context obtained
Making context current
GL_VENDOR=NVIDIA Corporation
GL_RENDERER=NVIDIA GeForce MX450/PCIe/SSE2
GL_VERSION=3.3.0 NVIDIA 560.35.03
GL_SHADING_LANGUAGE_VERSION=3.30 NVIDIA via Cg compiler
pthread_getconcurrency()=0
Version = 3.3.0 NVIDIA 560.35.03
Vendor = NVIDIA Corporation
Renderer = NVIDIA GeForce MX450/PCIe/SSE2
b3Printf: Selected demo: Physics Server
startThreads creating 1 threads.
starting thread 0
started thread 0 
MotionThreadFunc thread started
ven = NVIDIA Corporation
ven = NVIDIA Corporation
```