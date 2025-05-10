---
tags:
    - vscode
    - remote
    - ssh
    - vscode-server
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


---

## VSCode server offline

```bash
code --version
1.100.0
19e0f9e681ecb8e5c09d8784acaa601316ca4571
x64
```



```bash
https://update.code.visualstudio.com/commit:{COMMIT_ID}/server-linux-x64/stable
```

```bash
# Download vscode server
wget https://update.code.visualstudio.com/commit:19e0f9e681ecb8e5c09d8784acaa601316ca4571/server-linux-x64/stable
wget https://update.code.visualstudio.com/commit:19e0f9e681ecb8e5c09d8784acaa601316ca4571/server-linux-arm64/stable
```
