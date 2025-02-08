---
tags:
    - networking
    - tools
---

# Tools

## bmon
bmon is a monitoring and debugging tool to capture networking related statistics and prepare them visually in a human friendly way

```
sudo apt-get install bmon
```


## iftop
iftop is a network monitoring tool that shows a list of network connections in real-time

```bash
sudo apt-get install iftop
```

### example

```bash
sudo iftop -f "udp port <port_number>"

```