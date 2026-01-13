---
title: Mavlink custom message
tags:   
    - mavlink
    - custom
    - pymavlink
---

Create custom dialect and use it with pymavlink


```xml
<mavlink>
  <include>common.xml</include>

  <messages>
    <message id="42000" name="MY_CUSTOM_MSG">
      <description>Example custom message</description>
      <field type="uint32_t" name="foo">Foo counter</field>
      <field type="float" name="bar">Bar value</field>
    </message>
  </messages>
</mavlink>
```





### Generate 
Generate python model using `mavgen.py` installed by `pymavlink` package

!!! tip ""
    copy `common.xml`, `minimal.xml` and `standard.xml` to folder that contain our custom message

```bash
mavgen.py   \
    --lang Python \
    --wire-protocol 2.0 \
    --output mydialect.py \
    mydialect.xml
```

Copy the generate python module to pymavlink dialect v20 subfolder


### Demo: Send the Receive the custom message

```python
from pymavlink import mavutil
import time
from pymavlink.dialects.v20.common import MAVLink_message
from pymavlink.dialects.v20 import mydialect
from typing import cast
from multiprocessing import Process as mp

import os

os.environ["MAVLINK20"] = "1"
print(os.getenv("MAVLINK20"))


def sender():
    mavutil.set_dialect("mydialect")

    conn = mavutil.mavlink_connection("udpout:127.0.0.1:14550")
    conn = cast(mavutil.mavudp, conn)
    mav = cast(mydialect.MAVLink, conn.mav)
    i = 0
    while True:
        mav.my_custom_msg_send(i, 1.23)  # function name is lowercase
        i += 1
        time.sleep(1)


def listener():
    mavutil.set_dialect("mydialect")
    conn = mavutil.mavlink_connection(
        "udp:0.0.0.0:14550", source_system=42, source_component=200
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