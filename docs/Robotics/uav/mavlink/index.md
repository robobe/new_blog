---
title: Mavlink 
tags:
    - mavlink
    - pymavlink
---

MAVLink (Micro Air Vehicle Link) is a lightweight communication protocol used to exchange messages between drones (UAVs), their autopilots, ground control stations, and companion computers

In MAVLink, every message is tagged with two identifiers so the network knows who sent the message and which part of the system it came from:

<div class="grid-container">
    <div class="grid-item">
        <a href="pymavlink">
            <p>pymavlink</p>
        </a>
    </div>
</div>

## SYSID (System ID)
A number that identifies the vehicle/system. (1-255)
Distinguishes different vehicles on the same MAVLink network (Vehicle ID)

Examples:
- Drone #1 → SYSID = 1
- Drone #2 → SYSID = 2
- Simulator (SITL) → often SYSID = 1 (configurable)
- GCS () = 255

## COMPID (Component ID)
A number that identifies a component inside a system. (1-255)
Distinguishes subsystems within the same vehicle

Think of it as the process/module ID

Common COMPIDs:
- 1 → Autopilot (flight controller)
- 190 → Companion computer
- 191 → GCS (ground control station)
- 200+ → Custom components

## Source / Target / Broadcast
In MAVLink, every message has two layers of identity:

1️⃣ Message header → SOURCE
- These are set automatically by your MAVLink connection:
- source_system (your SYSID)
- source_component (your COMPID)

👉 Identifies who is sending the message.

2️⃣ Command fields → TARGET
- Certain messages (especially commands) include explicit target fields inside the payload:
- target_system
- target_component

👉 Identifies who should execute the command.

### Example: COMMAND_LONG(76)

![alt text](images/command_long.png)

### Broadcast

```
target_system    = 0
target_component = 0
```

- Telemetry → always broadcast
- Queries → sometimes broadcast
- Control commands → never broadcast unless you are 100% sure

---

## Mavlink network
MAVLink “stream” is usually **one physical link** to connect multiple customer we use tool like [**mavlink-router**](https://github.com/mavlink-router/mavlink-router)