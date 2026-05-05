import math
import time

from gz.transport13 import Node
from gz.msgs10.laserscan_pb2 import LaserScan

TOPIC = "/ultrasonic_lidar"


def lidar_cb(msg: LaserScan):
    valid_ranges = [
        r for r in msg.ranges
        if math.isfinite(r) and msg.range_min <= r <= msg.range_max
    ]

    if not valid_ranges:
        print("No object detected")
        return

    distance = min(valid_ranges)
    print(f"Distance to object: {distance:.3f} m")


def main():
    node = Node()

    ok = node.subscribe(LaserScan, TOPIC, lidar_cb)
    if not ok:
        raise RuntimeError(f"Failed to subscribe to {TOPIC}")

    print(f"Listening on {TOPIC}...")

    try:
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("Stopping")


if __name__ == "__main__":
    main()