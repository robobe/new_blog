---
title: Ardupilot mavlink with pymavlink
tags:
    - ardupilot
    - pymavlink
---



## SITL

```bash
# ver 4.6.1
./arducopter --model copter --defaults params/basic.param -I0
```

<details>
<summary>basic.param</summary>
```
--8<-- "docs/Robotics/uav/ardupilot/mavlink/pymavlink/code/basic.param"
```
</details>

---

<div class="grid-container">
    <div class="grid-item">
        <a href="#hello">
            <p>Hello</p>
        </a>
    </div>
    <div class="grid-item">
        <a href="#send-command-long-to-set-message-interval">
            <p>Set message interval</p>
        </a>
    </div>
    <div class="grid-item">
        <a href="#send-message-to-gcs">
            <p>Send message to GCS</p>
        </a>
    </div>
    <div class="grid-item">
        <a href="#send-other-location-using-adsb">
            <p>Send other location</p>
        </a>
    </div>
</div>

## Hello

Connect to SITL and print out every mavlink received
typically only heartbeat and timesync received on no one request other messages

```python
from pymavlink import mavutil
# cast helpers
from typing import cast
from pymavlink.dialects.v20.common import MAVLink_message
from pymavlink.mavutil import mavtcp


conn = mavutil.mavlink_connection('tcp:127.0.0.1:5760')
conn = cast(mavtcp, conn)
# Make sure the connection is valid
conn.wait_heartbeat()

# Get some information !
while True:
    try:
        msg = conn.recv_match(blocking=True)
        msg = cast(MAVLink_message, msg)
        if msg:
            print(msg.get_type(), msg.to_dict())
    except Exception as _:
        pass
```

---

## Send command long to set message interval

```python
--8<-- "docs/Robotics/uav/ardupilot/mavlink/pymavlink/code/request_message_interval.py"
```

## Send message to gcs

- Send message to GCS using **STATUSTEXT**
- [Mavlink](https://mavlink.io/en/messages/common.html#STATUSTEXT) info
- [Severity](https://mavlink.io/en/messages/common.html#MAV_SEVERITY)

```python
from pymavlink import mavutil
import time
from pymavlink.dialects.v20 import common

# Create the connection to the top-side computer as companion computer/autopilot
master = mavutil.mavlink_connection(
    "udpout:localhost:14550", source_system=1, source_component=190
)

while True:
    
    master.mav.statustext_send(
        common.MAV_SEVERITY_INFO, "QGC will read this".encode()
    )
    time.sleep(1)

```

!!! info "udpout"
    Send mavlink stream over udp


---

### Send other location using ADSB
Send other location information using ADSB check mavlink message [ADSB_VEHICLE (246) ](https://mavlink.io/en/messages/common.html#ADSB_VEHICLE)


```python
from pymavlink import mavutil
import time
from pymavlink.dialects.v20 import common


conn = mavutil.mavlink_connection("udpout:127.0.0.1:14550", source_system=0)

ICAO_ADDR = 0xABCDEF  # must be unique per vehicle

while True:
    conn.mav.adsb_vehicle_send(
        ICAO_ADDR,                 # ICAO address (unique ID)
        int(-35.362160 *1e7),        # lat (deg * 1e7)
        int(149.164975 * 1e7),        # lon (deg * 1e7)
        common.ADSB_ALTITUDE_TYPE_PRESSURE_QNH,
        100 * 1000,                # altitude (mm)
        0,                          # heading (cdeg)
        0,                          # horizontal velocity (cm/s)
        0,                          # vertical velocity (cm/s)
        b"OTHER_VEHICLE",            # callsign
        mavutil.mavlink.ADSB_EMITTER_TYPE_UAV,                          # emitter_type
        0,                          # tslc
        mavutil.mavlink.ADSB_FLAGS_VALID_COORDS,                           # flags
        0
    )
    time.sleep(1)

```
    
![alt text](images/asdb_location.png)