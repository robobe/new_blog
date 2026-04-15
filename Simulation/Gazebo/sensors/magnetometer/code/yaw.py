#!/usr/bin/env python3
import math
import time

from gz.transport13 import Node
from gz.msgs10.magnetometer_pb2 import Magnetometer


def magnetometer_cb(msg: Magnetometer):
    mx = msg.field_tesla.x
    my = msg.field_tesla.y
    mz = msg.field_tesla.z

    # Yaw from horizontal field only.
    # This is valid when the sensor is approximately level.
    yaw_rad = math.atan2(my, mx)
    yaw_deg = math.degrees(yaw_rad)

    if yaw_deg < 0:
        yaw_deg += 360.0

    print(
        f"mag: x={mx:.6e}, y={my:.6e}, z={mz:.6e} T | "
        f"yaw={yaw_deg:.2f} deg"
    )


def main():
    node = Node()
    topic = "/magnetometer"

    ok = node.subscribe(Magnetometer, topic, magnetometer_cb)
    if not ok:
        print(f"Failed to subscribe to {topic}")
        return

    print(f"Subscribed to {topic}")
    try:
        while True:
            time.sleep(0.01)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()