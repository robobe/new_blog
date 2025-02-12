---
tags:
    - networking
    - tips
---

# Networking Tips and Settings

## config machine as router
```
#!/bin/sh

sudo iptables -P FORWARD ACCEPT
sudo iptables -t nat -s 10.0.0.0/24 -A POSTROUTING -j MASQUERADE
echo 1 > sudo tee /proc/sys/net/ipv4/ip_forward
```

## Posts
- [checking mtu](checking_mtu.md)