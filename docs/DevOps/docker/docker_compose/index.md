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

---

## References
- [Ultimate Docker Compose Tutorial](https://youtu.be/SXwC9fSwct8)