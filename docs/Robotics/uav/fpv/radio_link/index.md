---
title: FPV Radio Link
tags:
    - drone
    - fpv
    - radio
    - receiver
---

## What It Is

The radio link is the control link between the pilot and the flight controller.

```text
Radio transmitter -> RF module -> receiver -> flight controller
```

It carries stick commands, arm switch, flight mode switch, and telemetry back from the aircraft.

<div class="grid-container">
    <div class="grid-item">
        <a href="expresslrs">
            <p>ExpressLRS</p>
        </a>
    </div>
    <div class="grid-item">
        <a href="crossfire">
            <p>TBS Crossfire</p>
        </a>
    </div>
    <div class="grid-item">
        <a href="tracer">
            <p>TBS Tracer</p>
        </a>
    </div>
    <div class="grid-item">
        <a href="frsky">
            <p>FrSky</p>
        </a>
    </div>
    <div class="grid-item">
        <a href="protocols">
            <p>Receiver Protocols</p>
        </a>
    </div>
</div>

## Main Choices

| System | Common band | Main strength | Typical protocol to FC |
| ------ | ----------- | ------------- | ---------------------- |
| `ExpressLRS` | `2.4 GHz` or `900 MHz` | Open source, low latency, long range | `CRSF` |
| `TBS Crossfire` | `868/915 MHz` | Long range and mature ecosystem | `CRSF` |
| `TBS Tracer` | `2.4 GHz` | Low latency control | `CRSF` |
| `FrSky` | `2.4 GHz` | Older common ecosystem | `SBUS`, `F.Port`, `S.Port` |
| `FlySky` | `2.4 GHz` | Low cost | `IBUS` |
| `Spektrum` | `2.4 GHz` | Common in RC aircraft | `SRXL2`, `DSM` |

## Selection Factors

- Range: long range builds usually prefer `900 MHz` systems.
- Latency: racing and freestyle benefit from faster packet rates.
- Telemetry: useful for battery voltage, link quality, GPS, and warnings.
- Antenna size: `900 MHz` antennas are larger than `2.4 GHz` antennas.
- Receiver size: micro builds need very small receivers.
- Ecosystem: transmitter module, receiver availability, firmware tools, and community support matter.
- Flight controller UARTs: most modern systems need a free UART.

## Related Pages

- [Video Link](../video_link/index.md)
- [Telemetry](../telemetry/index.md)
