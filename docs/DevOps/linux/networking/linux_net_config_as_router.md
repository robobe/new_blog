---
tags:
    - linux
    - networking
    - router
    - iptables
    - routing
---

# Linux as router

- eth0: WAN
- eth1: LAN

## Enable IP Forwarding
```bash title="sysctl"
sudo sysctl -w net.ipv4.ip_forward=1
```
- To make it permanent, add the following line to `/etc/sysctl.conf`:
```bash title="sysctl.conf"
net.ipv4.ip_forward = 1
```
- To apply the changes, run:
```bash title="sysctl"
sudo sysctl -p 
```

## IPTables Configuration
### rules
```bash title="iptables"
sudo iptables -t nat -A POSTROUTING -o eth0 -j MASQUERADE
sudo iptables -A FORWARD -i eth0 -o eth1 -j ACCEPT
sudo iptables -A FORWARD -i eth1 -o eth0 -m state --state ESTABLISHED,RELATED -j ACCEPT
```

### Save/Persist iptables rules

- To save the iptables rules, you can use the following command:
```bash title="iptables"
sudo iptables-save > /etc/iptables/rules.v4
```
- To restore the iptables rules on boot, you can use the following command:
```bash title="iptables"
sudo iptables-restore < /etc/iptables/rules.v4
```
- To make the iptables rules persistent across reboots, you can install the `iptables-persistent` package:
```bash title="apt"
sudo apt-get install iptables-persistent
```
- During the installation, you will be prompted to save the current iptables rules. Choose "Yes" to save them.
- After installation, the iptables rules will be automatically restored on boot.