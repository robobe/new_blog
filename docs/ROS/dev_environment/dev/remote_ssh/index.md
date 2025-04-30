---
tags:
    - ros
    - dev
    - vscode
    - remote
---

# Using VSCode remote


## ssh configuration

```ini title=".ssh/config"
Host myserver
  HostName 10.0.0.4
  Port 22
  User user
  ForwardX11 yes
  ForwardX11Trusted yes

```

