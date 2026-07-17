---
title: IP command cheat sheet
tags:
    - ip
    - networking
    - linux
    - cheat-sheet
---

<style>
    .ip-sheet {
        --ip-main: #35a58f;
        --ip-main-dark: #21806e;
        --ip-soft: #eaf7f3;
        --ip-row: #d8efe8;
        --ip-text: #20323a;
        color: var(--ip-text);
    }

    .ip-sheet h1 {
        color: var(--ip-main);
        font-size: 2.8rem;
        line-height: 1.05;
        margin: 0 0 1rem;
        font-weight: 800;
    }

    .ip-grid {
        display: grid;
        grid-template-columns: repeat(2, minmax(0, 1fr));
        gap: 1rem;
        align-items: start;
    }

    .ip-card {
        border: 1px solid #b7ddd4;
        background: #f8fcfb;
        border-radius: 6px;
        overflow: hidden;
        margin-bottom: 1rem;
    }

    .ip-card h2 {
        margin: 0;
        padding: 0.45rem 0.65rem;
        background: var(--ip-main);
        color: white;
        font-size: 1.15rem;
        line-height: 1.2;
    }

    .ip-card h3 {
        margin: 0;
        padding: 0.45rem 0.65rem;
        background: var(--ip-main-dark);
        color: white;
        font-size: 1rem;
        line-height: 1.2;
    }

    .ip-card table {
        width: 100%;
        border-collapse: collapse;
        margin: 0;
        font-size: 0.82rem;
    }

    .ip-card th,
    .ip-card td {
        padding: 0.42rem 0.55rem;
        vertical-align: top;
        border: 0;
    }

    .ip-card th {
        text-align: left;
        background: #f2faf7;
        color: #1b3840;
        font-weight: 800;
        border-bottom: 2px solid #8ccdc0;
    }

    .ip-card tr:nth-child(even) td {
        background: var(--ip-row);
    }

    .ip-card code {
        white-space: nowrap;
        font-size: 0.78rem;
    }

    .ip-note {
        border-radius: 6px;
        padding: 0.75rem 0.85rem;
        margin: 0 0 1rem;
        font-size: 0.88rem;
        border: 1px solid #b7ddd4;
        background: var(--ip-soft);
    }

    .ip-note strong {
        color: var(--ip-main-dark);
    }

    .ip-warning {
        border-radius: 6px;
        padding: 0.75rem 0.85rem;
        margin: 0 0 1rem;
        font-size: 0.88rem;
        border: 1px solid #f1b3a8;
        background: #ffe3de;
    }

    .ip-warning strong {
        color: #9c3328;
    }

    @media (max-width: 900px) {
        .ip-grid {
            grid-template-columns: 1fr;
        }

        .ip-sheet h1 {
            font-size: 2.1rem;
        }
    }
</style>

<div class="ip-sheet">

# IP Command Cheat Sheet

<div class="ip-grid">

<div>

<div class="ip-card">
<h2>Syntax</h2>

| Command | Description |
|---|---|
| `ip [options] OBJECT COMMAND` | General command form |
| `ip help` | Show global help |
| `ip OBJECT help` | Show help for one object |

</div>

<div class="ip-card">
<h2>IP Objects</h2>

| Object | Description |
|---|---|
| `addr` | IPv4 / IPv6 addresses on interfaces |
| `link` | Network interfaces |
| `route` | Routing table |
| `neigh` | Neighbor / ARP table |
| `rule` | Policy routing rules |
| `netns` | Network namespaces |
| `monitor` | Watch network changes |

</div>

<div class="ip-note">
<strong>Quick tip:</strong>
Many objects have short names. `addr` can be `a`, `link` can be `l`,
`route` can be `r`, and `neigh` can be `n`.
</div>

<div class="ip-card">
<h2>IP Options</h2>

| Option | Description |
|---|---|
| `-br` | Brief table output |
| `-c` | Colored output |
| `-4` | IPv4 only |
| `-6` | IPv6 only |
| `-s` | Show statistics |
| `-d` | Detailed output |
| `-j` | JSON output |
| `-p` | Pretty JSON output |

</div>

<div class="ip-card">
<h2>IP Command vs Net-Tools</h2>

| Net-tools | Modern command |
|---|---|
| `ifconfig` | `ip addr`, `ip link` |
| `route` | `ip route` |
| `arp` | `ip neigh` |
| `netstat -rn` | `ip route` |
| `netstat -tulpn` | `ss -tulpn` |

</div>

</div>

<div>

<div class="ip-card">
<h2>Manage IP Addresses</h2>

| Command | Description |
|---|---|
| `ip addr help` | Show address commands |
| `ip addr show` | Show all IP addresses |
| `ip -br -c addr` | Brief colored address view |
| `ip addr show dev eth0` | Show addresses on `eth0` |
| `sudo ip addr add 192.0.2.10/24 dev eth0` | Add an IP address |
| `sudo ip addr del 192.0.2.10/24 dev eth0` | Delete an IP address |
| `sudo ip addr flush dev eth0` | Remove all addresses from `eth0` |

</div>

<div class="ip-card">
<h2>Manage Network Interfaces</h2>

| Command | Description |
|---|---|
| `ip link help` | Show link commands |
| `ip link show` | Show all interfaces |
| `ip -br link` | Brief interface view |
| `ip link show dev eth0` | Show one interface |
| `sudo ip link set dev eth0 up` | Bring interface up |
| `sudo ip link set dev eth0 down` | Bring interface down |
| `sudo ip link set dev eth0 mtu 1400` | Change interface MTU |

</div>

<div class="ip-card">
<h2>Manage Routing Table</h2>

| Command | Description |
|---|---|
| `ip route help` | Show route commands |
| `ip route show` | Show routing table |
| `ip -c route` | Show colored routing table |
| `ip route show default` | Show default route |
| `ip route get 8.8.8.8` | Show route used for a destination |
| `sudo ip route add 10.10.0.0/16 via 192.168.1.1 dev wlan0` | Add route through gateway |
| `sudo ip route add 10.20.0.0/24 dev eth0` | Add directly connected route |
| `sudo ip route add default via 192.168.1.1 dev wlan0` | Add default route |
| `sudo ip route del 10.10.0.0/16 via 192.168.1.1 dev wlan0` | Delete network route |
| `sudo ip route del default` | Delete default route |

</div>

<div class="ip-card">
<h2>Manage Neighbor Entries</h2>

| Command | Description |
|---|---|
| `ip neigh help` | Show neighbor commands |
| `ip neigh` | Show neighbor / ARP table |
| `ip neigh show dev eth0` | Show entries for `eth0` |
| `sudo ip neigh add 192.168.1.50 lladdr aa:bb:cc:dd:ee:ff dev eth0 nud permanent` | Add static ARP entry |
| `sudo ip neigh replace 192.168.1.50 lladdr aa:bb:cc:dd:ee:ff dev eth0 nud permanent` | Add or update static ARP entry |
| `sudo ip neigh del 192.168.1.50 dev eth0` | Delete neighbor entry |

</div>

<div class="ip-card">
<h2>Useful Socket Commands</h2>

| Command | Description |
|---|---|
| `ss -lntp` | TCP listening sockets |
| `ss -natp` | TCP active connections |
| `ss -naup` | UDP sockets |
| `ss -lnp` | All listening sockets |

</div>

</div>

</div>

<div class="ip-warning">
<strong>Important:</strong>
Be careful when changing network interfaces, addresses, routes, and neighbor
entries on a remote machine. A wrong route, address, or interface state can
disconnect your SSH session. Test commands locally or keep a fallback console
when working on critical systems.
</div>

</div>
