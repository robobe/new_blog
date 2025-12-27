---
tags:
    - vscode
    - remote
    - ssh
    - vscode-server
---
The Remote - SSH extension lets you use any remote machine with a SSH server as your development environment.

[Check vscode remote and raspberry pi](docs/Embedded/RPI/ssh_vscode_remote)

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



