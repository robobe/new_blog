---
title: ip command
tags:
    - ip
    - networking
    - linux    
---

The ip command is the modern Linux networking utility provided by the iproute2 package. It replaces several older networking tools from the net-tools package

```bash
ip OBJECT COMMAND [OPTIONS]
```

- **OBJECT** specifies what you want to manage (address, link, route, neighbor, etc.).
- **COMMAND** specifies the action (show, add, delete, set, flush, etc.).

| Object    | Purpose                                        |
| --------- | ---------------------------------------------- |
| `addr`    | IP addresses assigned to interfaces            |
| `link`    | Network interfaces (Ethernet, Wi-Fi, loopback) |
| `route`   | Routing table                                  |
| `neigh`   | ARP/Neighbor cache                             |
| `rule`    | Policy routing rules                           |
| `monitor` | Watch networking changes in real time          |
| `netns`   | Network namespaces                             |
| `tunnel`  | GRE, IPIP, VXLAN, and other tunnels            |


| Command   | Description                                                 |
| --------- | ----------------------------------------------------------- |
| `show`    | Display information                                         |
| `add`     | Create or add an object                                     |
| `del`     | Delete an object                                            |
| `set`     | Modify an existing object                                   |
| `replace` | Add or replace an existing object                           |
| `flush`   | Remove all matching entries                                 |
| `get`     | Query information (for example, the route to a destination) |


### Demo

```bash
# all the command are same
# object: addr
# command: show
ip addr show
ip a s
ip a
```

## 10 most useful command

 1. ip addr show
 2. ip addr add
 3. ip addr del
 4. ip route show
 5. ip route add
 6. ip route del
 7. ip neigh
 8. ss -natp
 9. ss -naup
 10. ip link set


### ip addr show

```bash title="full object and command"
ip addr show
```

```bash title="brief address view with color"
ip -br -c a
```

### ip addr add

`ip addr add` adds an IP address to a network interface.

Basic form:

```bash
sudo ip addr add <ip>/<prefix> dev <interface>
```

Meaning:

- `<ip>/<prefix>` is the IP address and subnet size
- `dev <interface>` selects the network interface
- the command changes the running network configuration

Example:

```bash
sudo ip addr add 192.0.2.10/24 dev eth0
```

This adds the address `192.0.2.10` with subnet mask `255.255.255.0` to `eth0`.

Check it:

```bash
ip addr show dev eth0
```

### ip addr del

```bash
sudo ip addr del 192.0.2.10/24 dev eth0
```

This change is temporary. It is lost after reboot or network service restart
unless you also add it to the system network configuration.

---

### ip route show

`ip route show` displays the kernel routing table.

The routing table tells Linux where to send packets.

```bash
ip route show
```

Short form:

```bash
# output with color
ip -c r
```

Example output:

```text
default via 192.168.1.1 dev wlan0
192.168.1.0/24 dev wlan0 proto kernel scope link src 192.168.1.20
```

Meaning:

- `default via 192.168.1.1 dev wlan0`: traffic to unknown networks goes through gateway `192.168.1.1`
- `192.168.1.0/24 dev wlan0`: local LAN traffic goes directly through `wlan0`
- `src 192.168.1.20`: source IP used by this machine on that route

Show the route Linux will use for one destination:

```bash
ip route get 8.8.8.8
```

---

### ip route add

`ip route add` adds a route to a network.

Basic form:

```bash
sudo ip route add <network>/<prefix> via <gateway> dev <interface>
```

Example:

```bash
sudo ip route add 10.10.0.0/16 via 192.168.1.1 dev wlan0
```

This means:

- packets to `10.10.0.0/16`
- should be sent to gateway `192.168.1.1`
- through interface `wlan0`

Add a direct route without a gateway:

```bash
sudo ip route add 10.20.0.0/24 dev eth0
```

Use this when the target network is directly connected to `eth0`.

---

#### ip route add default

The default route is the route used when no more specific route matches.

It is usually the route to the internet through your router.

Long form:

```bash
sudo ip route add default via 192.168.1.1 dev wlan0
```

Equivalent form:

```bash
sudo ip route add 0.0.0.0/0 via 192.168.1.1 dev wlan0
```

Meaning:

- `default` means all destinations not matched by another route
- `via 192.168.1.1` is the next-hop router
- `dev wlan0` is the outgoing interface

Check it:

```bash
ip route show default
```

---

### ip route del

`ip route del` removes a route.

Delete a network route:

```bash
sudo ip route del 10.10.0.0/16 via 192.168.1.1 dev wlan0
```

Delete a direct route:

```bash
sudo ip route del 10.20.0.0/24 dev eth0
```

Delete the default route:

```bash
sudo ip route del default via 192.168.1.1 dev wlan0
```

Often this shorter command is enough:

```bash
sudo ip route del default
```

Route changes made with `ip route add` and `ip route del` are temporary. They
are lost after reboot or network service restart unless saved in the system
network configuration.

---

### ip neigh

`ip neigh` shows and manages the neighbor table.

For IPv4, this is the ARP table. It maps an IP address to a MAC address on the
local network.

Show neighbor entries:

```bash
ip neigh
# show result with color
ip -c neigh show
```

Show entries for one interface:

```bash
ip neigh show dev eth0
```

Example output:

```text
192.168.1.1 dev eth0 lladdr 00:11:22:33:44:55 REACHABLE
192.168.1.50 dev eth0 lladdr aa:bb:cc:dd:ee:ff STALE
```

Common states:

- `REACHABLE`: recently confirmed working
- `STALE`: known, but not recently confirmed
- `FAILED`: resolution failed
- `PERMANENT`: manually configured static entry

### ip neigh add

Add a static ARP entry:

```bash
sudo ip neigh add 192.168.1.50 lladdr aa:bb:cc:dd:ee:ff dev eth0 nud permanent
```

Meaning:

- `192.168.1.50` is the IP address
- `lladdr aa:bb:cc:dd:ee:ff` is the MAC address
- `dev eth0` selects the interface
- `nud permanent` makes it a static neighbor entry

Check it:

```bash
ip neigh show 192.168.1.50 dev eth0
```



---

## net-tools mapping

| Legacy Tool         | Modern Replacement   |
| ------------------- | -------------------- |
| `ifconfig`          | `ip addr`, `ip link` |
| `route`             | `ip route`           |
| `arp`               | `ip neigh`           |
| `netstat` (routing) | `ip route`           |
| `netstat` (sockets) | `ss`                 |


---

