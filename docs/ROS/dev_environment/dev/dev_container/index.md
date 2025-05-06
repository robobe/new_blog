---
tags:
    - ros
    - vscode
    - devcontainer
---

# VSCode Devcontainer for ROS application

## Project

```bash
.
├── .devcontainer
│   ├── devcontainer.json
│   ├── Dockerfile.humble_dev
│   ├── .tmux.conf
|   └── Dockerfile
├── docker-compose.yaml
├── readme.md
└── src
```

### Base docker
- ubuntu 22.04
- ros humble base
- gazebo11

<details>
    <summary>dockerfile.humble_dev</summary>

```Dockerfile
--8<-- "docs/ROS/dev_environment/dev/dev_container/code/dockerfile.humble_dev"
```
</details>

<details>
    <summary>build command</summary>

```bash
docker build -t humble:dev -f .devcontainer/Dockerfile.humble_dev .
```
</details>


### Dockerfile

```Dockerfile
--8<-- "docs/ROS/dev_environment/dev/dev_container/Dockerfile"
```

### Docker Compose

```yaml title="docker-compose.yml"
--8<-- "docs/ROS/dev_environment/dev/dev_container/docker-compose.yml"
```

### devcontainer

```json title="devcontainer.json"
--8<-- "docs/ROS/dev_environment/dev/dev_container/devcontainer.json"
```

|   |   |
|---|---|
| dockerComposeFile  |   |
| service | |
| shutdownAction | |
| workdpaceFolder |  |


### Other files

<details>
    <summary>.tmux.conf</summary>

```ini
--8<-- "docs/ROS/dev_environment/dev/dev_container/code/.tmux.conf"
```
</details>