---
tags:
    - ros
    - vscode
    - docker
    - devcontainer
    - docker compose
    - projects
---

# Ros2 projects templates

## ROS2 jezzy
```
├── .devcontainer
│   ├── devcontainer.json
│   ├── Dockerfile.jazzy
│   ├── Dockerfile.runtime
│   ├── Dockerfile.dev
│   └── Dockerfile
├── .vscode
│   ├── tasks.json
│   └── settings.json
├── docker-compose.yml
├── .gitignore
├── README.md
└── src
```

<details>
    <summary>.devcontainer/Dockerfile.jazzy</summary>

```json
--8<-- "docs/ROS/dev_environment/dev/project_templates/jazzy/code/Dockerfile.jazzy"
```
</details>

<details>
    <summary>.devcontainer/devcontainer.json</summary>

```json
--8<-- "docs/ROS/dev_environment/dev/project_templates/jazzy/code/devcontainer.json"
```
</details>


<details>
    <summary>docker-compose.yaml</summary>

```json
--8<-- "docs/ROS/dev_environment/dev/project_templates/jazzy/code/docker-compose.yaml"
```
</details>
