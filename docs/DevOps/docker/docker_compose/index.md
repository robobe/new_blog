---
tags:
    - docker
    - compose
---

# Docker Compose
Docker Compose is a tool for defining and managing multi-container Docker applications using a YAML configuration file. Instead of manually running multiple docker run commands

!!! note "`docker-compose` vs `docker compose`"
    `docker-compose` is the original command, `docker compose` is the new command. 
    The new command is available in Docker 1.29 and later.
    

## Usage     

```bash
docker compose -f docker-compose.yml up
docker compose -f docker-compose.yml run <service_name> <command>
```

<div class="grid-container">
    <div class="grid-item">
        <a href="compose_tips_and_templates">
        <img src="images/tips.png" width="150" height="150">
        <p>gui</p>
        </a>
    </div>
    <div class="grid-item">
    <a href="docker_compose">
        <img src="images/docker_compose.png" width="150" height="150">
        <p>TBD</p>
        </a>
    </div>
    
    
</div>

---

## compose override
Docker Compose override is a feature that lets you customize or extend a base docker-compose.yml file using additional YAML files like docker-compose.override.yml 

For example run nvidia support on pc and nvidia jetson

```yaml title="docker-compose.yaml"
services:
  dev:
    image: your-gpu-app:latest
    deploy:
      resources:
        reservations:
          devices:
            - capabilities: [gpu]

```

```yaml title="docker-compose.jetson.yaml"
services:
  dev:
    runtime: nvidia
    environment:
      - NVIDIA_VISIBLE_DEVICES=all
      - NVIDIA_DRIVER_CAPABILITIES=compute,utility

```

```bash
docker compose -f docker-compose.yml -f docker-compose.jetson.yml up
```

---

## References
- [Ultimate Docker Compose Tutorial](https://youtu.be/SXwC9fSwct8)