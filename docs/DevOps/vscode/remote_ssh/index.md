---
tags:
    - vscode
    - remote
    - ssh
---

# Remote ssh

## Tips
### Add x11 support

Add ForwardX11 and ForwardX11Trusted to user `.ssh/config` file

```
Host 10.0.0.4
  HostName 10.0.0.4
  User user
  ForwardX11 yes
  ForwardX11Trusted yes
```