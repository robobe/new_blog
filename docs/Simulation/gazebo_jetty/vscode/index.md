---
tags:
    - gazebo
    - gz
    - ionic
    - docker
    - vscode
---

# Run gazebo ionic on docker


## Dockerfile

<details>
    <summary>Docker file</summary>

```dockerfile
--8<-- "docs/Simulation/ionic/vscode/code/Dockerfile"
```
</details>

## Run the image

```bash
xhost +local:docker

docker run --gpus all -it --rm \
--name ionic \
--hostname gz \
--user user \
--network host \
--env="QT_X11_NO_MITSHM=1"  \
--env="DISPLAY"  \
-v /tmp/.X11-unix:/tmp/.X11-unix:rw \
-v /dev/dri:/dev/dri \
-v /dev/nvidia0:/dev/nvidia0 \
-v /dev/nvidiactl:/dev/nvidiactl \
-v /dev/nvidia-modeset:/dev/nvidia-modeset \
gz:ionic
```

---

## Use VSCode and docker
- Using docker-compose
- Using devcontainer to run the docker compose

```
├── .devcontainer
│   └── devcontainer.json
├── docker-compose.yaml
├── .gitignore
└── gz_tutorial
    └── worlds
        └── empty.world

```

```yaml title="docker-compose.yaml"
--8<-- "docs/Simulation/Gazebo/vscode/code/docker-compose.yaml"
```

```yaml title=".devcontainer/devcontainer.json"
--8<-- "docs/Simulation/Gazebo/vscode/code/devcontainer.json"
```

---