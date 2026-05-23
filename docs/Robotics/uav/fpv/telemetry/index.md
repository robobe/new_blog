---
title: FPV Telemetry
tags:
    - drone
    - fpv
    - telemetry
---

## What It Is

Telemetry is data sent from the aircraft back to the pilot, radio, goggles, ground station, or logging system.

Common telemetry data:

- Battery voltage
- Current draw and consumed mAh
- Link quality and RSSI
- GPS position
- Altitude
- Flight mode
- Warnings and failsafe status

## Common Paths

| Path | Example |
| ---- | ------- |
| Receiver telemetry | `CRSF`, `S.Port`, `F.Port`, `IBUS telemetry` |
| Video OSD | Betaflight OSD over analog or digital video |
| Ground station telemetry | `MAVLink` on ArduPilot or PX4 systems |
| VTX control data | `SmartAudio`, `IRC Tramp`, `MSP` |

## Practical Notes

- For Betaflight freestyle quads, the most visible telemetry is usually OSD in the goggles.
- For ExpressLRS and Crossfire, telemetry often goes back to the radio through `CRSF`.
- For long range, GPS position, link quality, and battery voltage are important.
- Telemetry should not replace a correct failsafe setup.

## Related Pages

- [Radio Link](../radio_link/index.md)
- [Video Link](../video_link/index.md)
