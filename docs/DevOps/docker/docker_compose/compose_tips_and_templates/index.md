---
tags:
    - docker
    - compose
    - tips
    - templates
---

# Docker compose tips and templates


## Add hostname to /etc/hosts
Used to manually define hostname-to-IP address mappings, similar to adding lines to the /etc/hosts file inside the container.

```yaml
hostname: dev
extra_hosts:
    - "dev:127.0.0.1"
```
