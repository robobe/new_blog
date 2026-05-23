---
title: FPV Video Link
tags:
    - drone
    - fpv
    - video
    - vtx
---

## What It Is

The video link sends the camera image from the aircraft to the pilot goggles or monitor.

```text
FPV camera -> VTX -> antenna -> goggles or receiver
```

## Main Types

| Type | Pros | Cons |
| ---- | ---- | ---- |
| Analog 5.8 GHz | Low latency, cheap, simple, light | Lower image quality |
| DJI digital FPV | High image quality, integrated ecosystem | More expensive, heavier hardware |
| Walksnail Avatar | Digital HD with many form factors | Ecosystem-specific hardware |
| HDZero | Digital with racing-focused low latency | Image quality and penetration depend strongly on setup |

## Selection Factors

- Latency: racing prefers very low latency.
- Image quality: cinematic flying usually prefers digital HD.
- Weight: tiny builds may need analog or lightweight digital hardware.
- Range and penetration: antenna choice and VTX power matter.
- Cooling: digital VTX units can overheat without airflow.
- Betaflight integration: VTX tables, SmartAudio, Tramp, or MSP display port may be needed.

## Related Pages

- [Radio Link](../radio_link/index.md)
- [Telemetry](../telemetry/index.md)
