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
