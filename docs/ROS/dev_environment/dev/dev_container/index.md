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
│   └── Dockerfile
├── docker-compose.yml
├── readme.md
└── src
```

### Docker Compose

```yaml title="docker-compose.yml"
--8<-- "docs/ROS/dev_environment/dev/dev_container/docker-compose.yml"
```

### Dockerfile

```Dockerfile
--8<-- "docs/ROS/dev_environment/dev/dev_container/Dockerfile"
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
