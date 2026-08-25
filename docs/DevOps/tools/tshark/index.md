---
title: tshark - Filter and Display Packet Fields
tags:
    - tshark
    - wireshark
    - tcpdump
    - networking
    - mavlink
---

`tshark` is Wireshark's command-line packet analyzer. It uses the same protocol
dissectors and display filters as Wireshark, but prints decoded packets in a
terminal or a script-friendly table.

## Install

```bash
sudo apt update
sudo apt install tshark tcpdump
```

## Capture MAVLink and print selected fields

This pipeline captures MAVLink UDP traffic on the loopback interface and sends
it directly to `tshark` without creating a capture file:

```bash
sudo tcpdump -i lo -nn -s 0 -U -w - 'udp dst port 14550' |
  tshark -l -r - \
    -d udp.port==14550,mavlink_proto \
    -T fields \
    -e frame.time_relative \
    -e mavlink_proto.msgid \
    -e _ws.col.Info \
    -E header=y -E separator=/t
```

The pipe separates capture from decoding:

- `tcpdump` captures only UDP packets whose destination port is `14550`.
- `tshark` decodes those packets as MAVLink and prints the requested fields.

### tcpdump options

| Option | Meaning |
| --- | --- |
| `-i lo` | Capture on the loopback interface. Replace `lo` with the required interface. |
| `-nn` | Do not resolve host names or service names. |
| `-s 0` | Capture each complete packet instead of truncating it. |
| `-U` | Write each packet to the pipe immediately. |
| `-w -` | Write pcap data to standard output. |
| `'udp dst port 14550'` | BPF capture filter: keep only UDP packets sent to port `14550`. |

### tshark options

| Option | Meaning |
| --- | --- |
| `-l` | Flush each output line immediately. |
| `-r -` | Read pcap data from standard input. |
| `-d udp.port==14550,mavlink_proto` | Decode UDP port `14550` as MAVLink. |
| `-T fields` | Print a table containing only selected fields. |
| `-e FIELD` | Add one field as an output column; repeat it for more columns. |
| `-E header=y` | Print the field names as a header row. |
| `-E separator=/t` | Separate columns with a tab. |

The example prints elapsed capture time, MAVLink message ID, and Wireshark's
Info column. Add or remove `-e` arguments to control the output:

```bash
-e frame.time_relative \
-e mavlink_proto.sysid \
-e mavlink_proto.compid \
-e mavlink_proto.msgid
```

List available MAVLink fields:

```bash
tshark -G fields | grep 'mavlink_proto\.'
```

## Add a tshark display filter

The quoted tcpdump expression is a **capture filter**: rejected packets never
reach `tshark`. Use `-Y` for a Wireshark **display filter** after packets have
been captured and decoded.

For example, add this option to show only MAVLink message ID `0`:

```bash
-Y 'mavlink_proto.msgid == 0'
```

The relevant part of the command becomes:

```bash
tshark -l -r - \
  -d udp.port==14550,mavlink_proto \
  -Y 'mavlink_proto.msgid == 0' \
  -T fields \
  -e frame.time_relative \
  -e mavlink_proto.msgid \
  -e _ws.col.Info \
  -E header=y -E separator=/t
```

Use the capture filter to reduce incoming traffic, the display filter to select
decoded packets, and `-e` fields to choose the columns displayed.

