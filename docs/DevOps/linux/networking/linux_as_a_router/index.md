---
title: Configure Linux as a router
tags:
    - linux
    - networking
    - router
    - iptables
    - nat
---

# Configure Linux as a router

This page shows the minimal setup for using a Linux box as a router between two
network interfaces.

Example topology:

```text
LAN clients <----> eth1 [ Linux router ] eth0 <----> WAN / internet
```

- `WAN`: interface connected to the upstream network or internet
- `LAN`: interface connected to the local/private network

Check your interface names:

```bash
ip -br link
ip -br addr
```

---

## Enable IPv4 forwarding

Enable forwarding immediately:

```bash
sudo sysctl -w net.ipv4.ip_forward=1
```

Make it persistent:

```bash
echo "net.ipv4.ip_forward = 1" | sudo tee /etc/sysctl.d/99-router.conf
sudo sysctl --system
```

Check it:

```bash
sysctl net.ipv4.ip_forward
```

Expected:

```text
net.ipv4.ip_forward = 1
```

---

## Generate iptables router rules

Enter the interface connected to the WAN and the interface connected to the LAN.
Choose whether to insert rules at the top of the chains or append them at the
end.

Default recommendation: **insert at the top**. This makes the router rules run
before older rules in the same chain.

<div class="router-rule-tool">
    <style>
        .router-rule-tool {
            border: 1px solid #b7ddd4;
            border-radius: 8px;
            padding: 1rem;
            margin: 1rem 0;
            background: #f8fcfb;
        }

        .router-rule-tool label {
            display: block;
            font-weight: 700;
            margin: 0.7rem 0 0.25rem;
        }

        .router-rule-tool input[type="text"] {
            width: 100%;
            max-width: 22rem;
            padding: 0.45rem 0.55rem;
            border: 1px solid #8fcbbf;
            border-radius: 5px;
            font: inherit;
        }

        .router-rule-tool .mode-row {
            display: flex;
            flex-wrap: wrap;
            gap: 1rem;
            margin: 0.5rem 0 1rem;
        }

        .router-rule-tool .mode-row label {
            display: inline-flex;
            gap: 0.35rem;
            align-items: center;
            margin: 0;
            font-weight: 600;
        }

        .router-rule-tool button {
            margin: 0.5rem 0 0.75rem;
            padding: 0.5rem 0.8rem;
            border: 0;
            border-radius: 5px;
            background: #35a58f;
            color: white;
            font-weight: 700;
            cursor: pointer;
        }

        .router-rule-tool textarea {
            width: 100%;
            min-height: 18rem;
            padding: 0.75rem;
            border: 1px solid #8fcbbf;
            border-radius: 5px;
            font-family: ui-monospace, SFMono-Regular, Menlo, Consolas, monospace;
            font-size: 0.85rem;
            line-height: 1.45;
        }
    </style>

    <label for="router-wan-iface">WAN interface</label>
    <input id="router-wan-iface" type="text" value="eth0" placeholder="eth0, enp3s0, wlan0">

    <label for="router-lan-iface">LAN interface</label>
    <input id="router-lan-iface" type="text" value="eth1" placeholder="eth1, enp4s0">

    <label>Rule placement</label>
    <div class="mode-row">
        <label>
            <input type="radio" name="router-rule-mode" value="insert" checked>
            Insert at top
        </label>
        <label>
            <input type="radio" name="router-rule-mode" value="append">
            Append at end
        </label>
    </div>

    <button type="button" onclick="generateRouterRules()">Generate commands</button>

    <textarea id="router-rule-output" spellcheck="false"></textarea>

    <script>
        function generateRouterRules() {
            const wan = document.getElementById("router-wan-iface").value.trim() || "eth0";
            const lan = document.getElementById("router-lan-iface").value.trim() || "eth1";
            const mode = document.querySelector('input[name="router-rule-mode"]:checked').value;
            const insert = mode === "insert";

            const natRule = insert
                ? `sudo iptables -t nat -I POSTROUTING 1 -o ${wan} -j MASQUERADE`
                : `sudo iptables -t nat -A POSTROUTING -o ${wan} -j MASQUERADE`;

            const forwardLanToWan = insert
                ? `sudo iptables -I FORWARD 1 -i ${lan} -o ${wan} -j ACCEPT`
                : `sudo iptables -A FORWARD -i ${lan} -o ${wan} -j ACCEPT`;

            const forwardWanToLanEstablished = insert
                ? `sudo iptables -I FORWARD 2 -i ${wan} -o ${lan} -m conntrack --ctstate ESTABLISHED,RELATED -j ACCEPT`
                : `sudo iptables -A FORWARD -i ${wan} -o ${lan} -m conntrack --ctstate ESTABLISHED,RELATED -j ACCEPT`;

            const output = `# Enable IPv4 forwarding now
sudo sysctl -w net.ipv4.ip_forward=1

# Make IPv4 forwarding persistent
echo "net.ipv4.ip_forward = 1" | sudo tee /etc/sysctl.d/99-router.conf
sudo sysctl --system

# NAT LAN traffic out through the WAN interface
${natRule}

# Allow LAN clients to go out through WAN
${forwardLanToWan}

# Allow return traffic from WAN back to LAN
${forwardWanToLanEstablished}

# Check rules
sudo iptables -t nat -L POSTROUTING -v -n --line-numbers
sudo iptables -L FORWARD -v -n --line-numbers
`;

            document.getElementById("router-rule-output").value = output;
        }

        generateRouterRules();
    </script>
</div>

---

## What the generated rules do

### NAT / masquerade

```bash
sudo iptables -t nat -I POSTROUTING 1 -o eth0 -j MASQUERADE
```

This rewrites LAN client source addresses to the router WAN address when traffic
leaves through the WAN interface.

Use this when the WAN address is dynamic, such as DHCP or Wi-Fi.

### Forward LAN to WAN

```bash
sudo iptables -I FORWARD 1 -i eth1 -o eth0 -j ACCEPT
```

This allows packets from the LAN interface to leave through the WAN interface.

### Allow return traffic

```bash
sudo iptables -I FORWARD 2 -i eth0 -o eth1 -m conntrack --ctstate ESTABLISHED,RELATED -j ACCEPT
```

This allows replies for connections that LAN clients started.

It does not allow random new incoming traffic from WAN to LAN.

---

## Insert vs append

| Mode | Command | When to use |
|---|---|---|
| insert | `-I CHAIN 1` | Put the rule near the top before older rules |
| append | `-A CHAIN` | Put the rule at the end after existing rules |

For firewall chains, order matters. The first matching rule usually decides what
happens to the packet.

---

## Save rules

Rules added with `iptables` are usually temporary.

Install persistent rule support on Debian/Ubuntu:

```bash
sudo apt install iptables-persistent
```

Save current IPv4 rules:

```bash
sudo iptables-save | sudo tee /etc/iptables/rules.v4
```

Restore manually:

```bash
sudo iptables-restore < /etc/iptables/rules.v4
```

---

## Remove generated rules

List rules with line numbers:

```bash
sudo iptables -t nat -L POSTROUTING -v -n --line-numbers
sudo iptables -L FORWARD -v -n --line-numbers
```

Delete by line number:

```bash
sudo iptables -t nat -D POSTROUTING <line-number>
sudo iptables -D FORWARD <line-number>
```

Delete carefully. Line numbers change after each delete.

!!! warning "Remote access"
    Be careful when changing routing and firewall rules over SSH. A wrong
    interface name, forwarding rule, or default policy can disconnect the
    machine.
