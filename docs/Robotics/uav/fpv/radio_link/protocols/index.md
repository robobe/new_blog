---
title: Receiver Protocols
tags:
    - drone
    - fpv
    - receiver
    - protocol
---

## Control Protocols

The RF system and the flight-controller receiver protocol are related but not the same thing.

Example:

```text
ExpressLRS is the radio system.
CRSF is the protocol used between the receiver and Betaflight.
```

| Protocol | Typical systems | Notes |
| -------- | --------------- | ----- |
| `CRSF` | ExpressLRS, Crossfire, Tracer | Fast serial protocol with control and telemetry |
| `SBUS` | FrSky, Futaba-style receivers | Common older one-way control protocol |
| `F.Port` | FrSky | Combines control and telemetry |
| `IBUS` | FlySky | Simple serial receiver protocol |
| `SRXL2` | Spektrum | Spektrum serial protocol |
| `PWM` | old receivers | One wire per channel, not common for modern FPV |
| `PPM` | old receivers | Multiple channels on one signal wire, mostly obsolete |

## Betaflight Setup Pattern

For most serial receivers:

1. Wire the receiver to a free UART.
2. Enable `Serial RX` on that UART.
3. Select the matching receiver protocol.
4. Check channel movement in the Receiver tab.
5. Set failsafe before flying.

