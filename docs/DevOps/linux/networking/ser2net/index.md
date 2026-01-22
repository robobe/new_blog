---
title: ser2net
tags:
    - ser2net
    - tools
    - networking
---

ser2net is a daemon that exposes serial ports over the network (TCP/UDP), turning /dev/tty* devices into network-accessible endpoints.

```bash
sudo ser2net -d -c config.yaml
```

<details>
<summary>config</summary>
```
--8<-- "docs/DevOps/linux/networking/ser2net/code/config.yaml"
```
</details>