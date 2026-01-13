---
title: Mavlink V2_EXTENSION message
tags:
    - mavlink
---
In MAVLink v2, devices sometimes need to send data that the receiver does not have a dialect for.
Wrap an unknown/custom message inside a standard MAVLink v2 message so that all routers and tools can forward it safely.

There no define pattern but there are strong conventions 

- Use little-endian
- Same field ordering as .xml
- No MAVLink header or CRC
- message_type = original MAVLink msg ID


```python
import struct

# version=1, counter=42, value=3.14
payload = struct.pack("<BIf", 1, 42, 3.14)
payload = payload.ljust(249, b"\x00")

```

## Demo

```python
from __future__ import annotations

import struct
import time
from multiprocessing import Process as mp
from typing import cast

from pymavlink import mavutil
from pymavlink.dialects.v20 import common
from pymavlink.dialects.v20.common import MAVLink_message


# ---- Custom V2_EXTENSION payload spec (example) ----
# payload layout (little-endian):
#   uint8   version
#   uint32  counter
#   float32 value
#
# total used bytes: 1 + 4 + 4 = 9
# remaining bytes padded with 0 up to 249


CUSTOM_MESSAGE_TYPE = 42000  # your private "embedded message id"
PAYLOAD_VERSION = 1


def pack_custom_payload(counter: int, value: float) -> bytes:
    raw = struct.pack("<BIf", PAYLOAD_VERSION, counter, value)  # 9 bytes
    if len(raw) > 249:
        raise ValueError("Payload too large")
    return raw.ljust(249, b"\x00")


def unpack_custom_payload(payload: bytes) -> tuple[int, int, float]:
    # payload can be longer; we only care about first 9 bytes
    version, counter, value = struct.unpack("<BIf", payload[:9])
    return version, counter, value


def sender() -> None:
    conn = mavutil.mavlink_connection(
        "udpout:127.0.0.1:14550",
        source_system=43,
        source_component=201,
        force_mavlink1=False,  # force MAVLink v2
    )
    conn = cast(mavutil.mavudp, conn)
    mav = cast(common.MAVLink, conn.mav)

    counter = 0
    while True:
        payload = pack_custom_payload(counter, value=3.14)

        # V2_EXTENSION is in the common dialect
        mav.v2_extension_send(
            target_network=0,          # commonly 0 (used for routing in some stacks)
            target_system=0,           # 0 = broadcast, or set to specific sysid
            target_component=0,        # 0 = broadcast, or set to specific compid
            message_type=CUSTOM_MESSAGE_TYPE,
            payload=payload,
        )

        counter += 1
        time.sleep(1)


def listener() -> None:
    conn = mavutil.mavlink_connection(
        "udp:0.0.0.0:14550",
        source_system=42,
        source_component=200,
        force_mavlink1=False,  # accept v2 (and don't force v1)
    )

    print("Listening on udp:0.0.0.0:14550")

    while True:
        msg = conn.recv_match(blocking=True)
        if not msg:
            continue

        msg = cast(MAVLink_message, msg)

        if msg.get_type() == "V2_EXTENSION":
            # msg.payload is typically a bytes-like object (may be list/bytearray in some builds)
            payload_bytes = bytes(getattr(msg, "payload", b""))
            mtype = getattr(msg, "message_type", None)

            if mtype == CUSTOM_MESSAGE_TYPE:
                version, counter, value = unpack_custom_payload(payload_bytes)
                print(f"V2_EXTENSION custom: type={mtype} v={version} counter={counter} value={value}")
            else:
                print(f"V2_EXTENSION other: type={mtype}")
        else:
            # still print other messages if any
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