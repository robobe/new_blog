---
tags:
    - linux
    - ip
    - cheat sheet
---

# IP command cheat sheet


|   |   |
|---|---|
| set interface ip  | ip addr add 192.168.1.1/24 dev eth0  |
| set MTU  | ip link set eth0 mtu 1400 |
| up / down | ip link set eth0 up, ip link set eth0 down |


## Route

|   |   |
|---|---|
| Adding a default gateway  | ip route add default via 192.168.1.254  |
