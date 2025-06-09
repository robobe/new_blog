---
tags:
    - vscode
    - devcontainer
    - volume
    - docker
---

# VSCode docker volume
Using devcontainer vscode save `vscode-server` on docker volume to share between different containers

It save the `vscode-server` binary in `/vscode/vscode-server/bin/linux-x64/<commit id>`, each time we upgrade the vscode it add new `<commit id>` folder


```bash title="list servers"
docker run -it --rm -v vscode:/vscode busybox ls /vscode/vscode-server/bin/linux-x64
```

!!! tip "vscode version and commit id"
    ```
    code -v
    ```

    ```
    code -v | head -n 2 | tail -n 1
    ```
     


## backup

```bash
ver=$(code -v | head -n 1)
docker run --rm \
  -v vscode:/vscode \
  -v $(pwd):/backup \
  busybox \
  sh -c "tar czf /backup/vscode_volume_${ver}.tar.gz -C /vscode ."
```


## Restore

```bash
ver=$(code -v | head -n 1)
docker run --rm \
  -v vscode:/vscode \
  -v $(pwd):/backup \
  busybox \
  sh -c "tar xzf /backup/vscode_volume_${ver}.tar.gz -C /vscode"

```

## Clean volume
```bash title="remove all vscode-server old folder"
commit_id=$(code -v | head -n 2 | tail -n 1)
docker run -it --rm \
-v vscode:/vscode busybox \
busybox \
sh -c "cd /vscode/vscode-server/bin/linux-x64 && for d in *; do [ \"\$d\" != \"${commit_id}\" ] && [ -d \"\$d\" ] && rm -rf \"\$d\"; done"


```