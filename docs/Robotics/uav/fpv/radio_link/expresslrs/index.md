---
title: ExpressLRS
tags:
    - drone
    - fpv
    - expresslrs
    - elrs
---

## Overview

`ExpressLRS`, often written `ELRS`, is an open-source FPV radio control system. It is popular because it gives strong range, low latency, small receivers, and many hardware options.

Common bands:

- `2.4 GHz`: common for freestyle, racing, and general FPV.
- `868/915 MHz`: common for long range, depending on local radio regulations.

## Pros And Cons

| Pros | Cons |
| ---- | ---- |
| Open source ecosystem | Firmware setup can confuse beginners |
| Very good range and latency | Hardware must use compatible firmware targets |
| Many receivers and transmitter modules | Region and frequency settings must be correct |
| Uses `CRSF` protocol to the flight controller | Some tiny receivers have limited telemetry power |

## Betaflight Connection

ExpressLRS receivers normally connect to a UART and use `CRSF`:

```text
Receiver TX -> Flight controller RX
Receiver RX -> Flight controller TX
5V or 3.3V power, depending on receiver
GND -> GND
```

In Betaflight:

- Ports tab: enable `Serial RX` on the receiver UART.
- Receiver tab: set receiver protocol to `CRSF`.

See [CRSF Protocol](../protocols/crsf/index.md) for the frame structure, message types, device addresses, and how ExpressLRS relates to Crossfire.

### Demo: ExpressLRS sniffer

- bind expresslrs receiver
- connect it to serial2usb (tx->rx)

<details>
<summary>Express LRS sniffer</summary>
```
--8<-- "docs/Robotics/uav/fpv/radio_link/expresslrs/code/elrs_crsf_sniffer.py"
```
</details>
