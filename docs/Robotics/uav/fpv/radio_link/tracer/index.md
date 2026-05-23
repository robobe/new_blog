---
title: TBS Tracer
tags:
    - drone
    - fpv
    - tracer
    - tbs
---

## Overview

`TBS Tracer` is a 2.4 GHz radio link focused on low latency control. It is closer to racing and freestyle use than classic long-range Crossfire.

## Pros And Cons

| Pros | Cons |
| ---- | ---- |
| Low latency | Less common than ExpressLRS |
| Uses `CRSF` with Betaflight | Shorter practical range than 900 MHz long-range systems |
| Smaller antennas than 900 MHz | Smaller ecosystem than ELRS |
| Good for responsive control | Requires compatible TBS hardware |

## Betaflight Connection

Tracer uses `CRSF`, so the flight controller setup is similar to Crossfire:

```text
Receiver TX -> Flight controller RX
Receiver RX -> Flight controller TX
5V -> 5V
GND -> GND
```

