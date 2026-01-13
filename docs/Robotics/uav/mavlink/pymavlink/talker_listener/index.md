---
title: pymavlink send message between two process
tags:
    - pymavlink
---

### Send mavlink message between two process

```python
from pymavlink import mavutil
import time
from pymavlink.dialects.v20.common import MAVLink_message
from pymavlink.dialects.v20 import common
from typing import cast
from multiprocessing import Process as mp


def sender():
    conn = mavutil.mavlink_connection(
    "udpout:127.0.0.1:14550",
    source_system=43,
    source_component=201
    )
    conn = cast(mavutil.mavudp, conn)
    mav = cast(common.MAVLink, conn.mav)

    while True:
        mav.heartbeat_send(
            common.MAV_TYPE_GENERIC,
            common.MAV_AUTOPILOT_INVALID,
            0, 0, 0
        )
        time.sleep(1)

def listener():
    conn = mavutil.mavlink_connection(
    "udp:0.0.0.0:14550",
    source_system=42,
    source_component=200
)

    print("Listening on udp:0.0.0.0:14550")
    
    while True:
        msg = conn.recv_match(blocking=True)
        msg = cast(MAVLink_message, msg)
        if msg:
            print(msg.get_type(), msg.to_dict())

if __name__ == "__main__":
    
    talker = mp(target=sender)
    listen = mp(target=listener)
    
    listen.start()
    time.sleep(1)  # Ensure listener starts before talker
    talker.start()
    talker.join()
    listen.join()
        
```