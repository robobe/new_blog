---
tags:
    - ros
    - deploy
    - tutorial
---

# ROS2 project from development to deployment, Part 4: Using docker for cross compiler

- Build docker image for ARM 
- Using github action to run build process on the docker
    - using `act` to run locally
  - 
## Build docker image
Run ARM docker architecture on x64 machine [install](docs/ROS/dev_environment/build/ros_build_using_docker_cross_compile.md)

<details>
    <summary>Docker</summary>
```Dockerfile
{{PLUGIN uri="/home/user/workspaces/deploy_demo_ws/Docker/Dockerfile.arm"}}
```

</details>


```
docker build -t ros2_cross_compile:arm64v8 -f Docker/Dockerfile.arm .
```

---

## Using github actions
### Using github actions local using `act`

```bash title="install act"
curl --proto '=https' --tlsv1.2 -sSf https://raw.githubusercontent.com/nektos/act/master/install.sh | sudo bash

```

### Config github action in project

- Add `.github` folder
- Add `workflows` folder
- Add action yaml file

```bash title=".github action folder and files"
.github/
└── workflows
    ├── build.yml
    └── README.md

```

```yaml title="build.yaml" linenums="1" hl_lines="25-27"
{{PLUGIN uri="/home/user/workspaces/deploy_demo_ws/.github/workflows/build.yml"}}
```

!!! tip "update rosdep sources"
    Update docker `/etc/ros/rosdep/source.list.d` with project custom DB and run `rosdep update` 
     

```bash title="run"
act -j build_pkg_interface -P arm=ros2_cross_compile:arm64v8 \
    --pull=false \
    --bind --directory . 
```