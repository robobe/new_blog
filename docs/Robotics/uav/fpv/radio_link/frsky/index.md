---
title: FrSky
tags:
    - drone
    - fpv
    - frsky
    - receiver
---

## Overview

`FrSky` systems were very common in older FPV builds. Common families include `ACCST` and `ACCESS`. They are usually 2.4 GHz systems.

## Common Protocols

| Protocol | Use |
| -------- | --- |
| `SBUS` | Receiver control signal to flight controller |
| `S.Port` | Telemetry back to radio |
| `F.Port` | Combined control and telemetry on one wire |

## Pros And Cons

| Pros | Cons |
| ---- | ---- |
| Many older radios and receivers exist | Firmware compatibility can be confusing |
| Small receiver options | Usually not the first choice for new long-range builds |
| Betaflight support is mature | Telemetry setup is less simple than `CRSF` systems |

## Betaflight Connection

For `SBUS`, use an SBUS-capable pad or inverted UART input if the flight controller requires it.

```text
Receiver SBUS -> Flight controller SBUS/RX pad
5V -> 5V
GND -> GND
```

