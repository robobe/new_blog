---
title: tcpdump
tags:
    - linux
    - networking
    - tcpdump
    - iftop
    - bandwidth
    - pcap
---

# tcpdump

`tcpdump` is a command-line packet capture tool.

It listens on a network interface and prints packets that match a filter.

Use it when you need to debug:

- traffic reaching or leaving a machine
- UDP/TCP ports
- source and destination addresses
- DNS, ICMP, ARP, DHCP, and other network protocols
- routing, firewall, or NAT problems

Basic capture:

```bash
sudo tcpdump -i eth0
```

Meaning:

- `sudo`: packet capture usually needs root permissions
- `-i eth0`: capture packets on interface `eth0`

Show available interfaces:

```bash
sudo tcpdump -D
```

Capture on any interface:

```bash
sudo tcpdump -i any
```

## Useful basic options

```bash
sudo tcpdump -i eth0 -nn -vv
```

Common options:

- `-i <interface>`: select network interface
- `-n`: do not resolve host names
- `-nn`: do not resolve host names or port names
- `-v`, `-vv`, `-vvv`: show more packet details
- `-c <count>`: stop after a number of packets
- `-s 0`: capture the full packet, not only the default snapshot size
- `-X`: print packet data in hex and ASCII
- `-A`: print packet data as ASCII

Example:

```bash
sudo tcpdump -i eth0 -nn -c 10
```

This captures 10 packets and then stops.

## Save packets to a pcap file

Use `-w` to write packets to a `.pcap` file.

```bash
sudo tcpdump -i eth0 -nn -s 0 -w capture.pcap
```

Important:

- `-w capture.pcap` saves packets to a file
- `-s 0` captures the full packet
- the terminal will not print decoded packets while writing with `-w`

Stop the capture with `Ctrl+C`.

You can later open the file with:

- `tcpdump`
- Wireshark
- tshark

## Read a pcap file

Use `-r` to read packets from a saved `.pcap` file.

```bash
tcpdump -nn -r capture.pcap
```

Read with more details:

```bash
tcpdump -nn -vv -r capture.pcap
```

Read and print packet payload:

```bash
tcpdump -nn -A -r capture.pcap
```

You can also apply filters while reading:

```bash
tcpdump -nn -r capture.pcap udp port 5600
```

## Packet filters

tcpdump uses BPF filters.

A filter selects which packets to capture or display.

Common filter parts:

- protocol: `tcp`, `udp`, `icmp`, `arp`
- host: `host 192.168.1.10`
- source host: `src host 192.168.1.10`
- destination host: `dst host 192.168.1.20`
- port: `port 5600`
- source port: `src port 5600`
- destination port: `dst port 5600`
- network: `net 192.168.1.0/24`

Examples:

```bash
sudo tcpdump -i eth0 -nn udp
```

```bash
sudo tcpdump -i eth0 -nn host 192.168.1.10
```

```bash
sudo tcpdump -i eth0 -nn tcp port 80
```

```bash
sudo tcpdump -i eth0 -nn icmp
```

## Filter with multiple conditions

Use `and`, `or`, and `not` to combine conditions.

Example: capture UDP traffic on port `5600` from source address
`192.168.1.50`:

```bash
sudo tcpdump -i eth0 -nn 'udp and src host 192.168.1.50 and port 5600'
```

More specific version: source address `192.168.1.50`, UDP destination port
`5600`:

```bash
sudo tcpdump -i eth0 -nn 'udp and src host 192.168.1.50 and dst port 5600'
```

Save the same filtered traffic to a pcap file:

```bash
sudo tcpdump -i eth0 -nn -s 0 \
  'udp and src host 192.168.1.50 and dst port 5600' \
  -w udp_5600_from_192_168_1_50.pcap
```

Read only matching packets from the pcap:

```bash
tcpdump -nn -r udp_5600_from_192_168_1_50.pcap \
  'udp and src host 192.168.1.50 and dst port 5600'
```

Example with `or`:

```bash
sudo tcpdump -i eth0 -nn 'udp port 5600 or udp port 14550'
```

Example with `not`:

```bash
sudo tcpdump -i eth0 -nn 'not port 22'
```

This avoids showing SSH traffic while you debug from a remote shell.

## Common tcpdump usages to learn

Important topics to know:

- capture by interface with `-i`
- disable name resolution with `-nn`
- limit packet count with `-c`
- save and read pcap files with `-w` and `-r`
- capture full packets with `-s 0`
- filter by protocol, host, network, and port
- combine filters with `and`, `or`, `not`
- inspect payloads with `-A` and `-X`
- debug DNS with `udp port 53`
- debug ping with `icmp`
- debug ARP with `arp`
- debug TCP handshakes with `tcp`
- avoid capturing SSH noise with `not port 22`
- open pcap files in Wireshark for visual analysis

---

## iftop bandwidth by port {#iftop-bandwidth}

`tcpdump` captures packets. For live bandwidth or traffic rate, `iftop` is often
easier.

`iftop` shows live traffic between hosts and can use packet filters like
`tcpdump`.

Install:

```bash
sudo apt install iftop
```

Measure traffic for a specific port:

```bash
sudo iftop -i eth0 -f 'port 5600'
```

Only UDP port `5600`:

```bash
sudo iftop -i eth0 -f 'udp port 5600'
```

Only UDP traffic from source IP `192.168.1.50` to destination port `5600`:

```bash
sudo iftop -i eth0 -n -P \
  -f 'udp and src host 192.168.1.50 and dst port 5600'
```

Useful options:

- `-i eth0`: listen on interface `eth0`
- `-n`: do not resolve hostnames
- `-P`: show ports
- `-f '<filter>'`: apply a packet filter

In the `iftop` screen:

- `=>` and `<=` show traffic direction
- the rate columns show recent traffic averages
- the columns are commonly `2s`, `10s`, and `40s` averages

Use `iftop` when you want a live bandwidth view. Use `tcpdump` when you need to
save packets, inspect packet contents, or debug protocol details.

---

## Quick cheat sheet

```bash
# Capture on eth0
sudo tcpdump -i eth0 -nn

# Capture 20 packets
sudo tcpdump -i eth0 -nn -c 20

# Capture UDP port 5600
sudo tcpdump -i eth0 -nn 'udp port 5600'

# Capture from one source address
sudo tcpdump -i eth0 -nn 'src host 192.168.1.50'

# Capture UDP destination port 5600 from one source
sudo tcpdump -i eth0 -nn 'udp and src host 192.168.1.50 and dst port 5600'

# Save to pcap
sudo tcpdump -i eth0 -nn -s 0 -w capture.pcap

# Read pcap
tcpdump -nn -r capture.pcap

# Read pcap and filter
tcpdump -nn -r capture.pcap 'udp port 5600'
```
