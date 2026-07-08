#!/usr/bin/env python3
"""Send RC commands to Betaflight SITL via UDP port 9004."""

import socket
import struct
import time

SITL_RC_PORT = 9004
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def send_rc(channels):
    timestamp = time.time()
    # Pack: double timestamp + 16 x uint16_t channels
    data = struct.pack('<d', timestamp)
    for ch in channels:
        data += struct.pack('<H', ch)
    sock.sendto(data, ('127.0.0.1', SITL_RC_PORT))

# Channel mapping (adjust to match your mode assignments):
#   CH1=Roll, CH2=Pitch, CH3=Throttle, CH4=Yaw, CH5=ARM, CH6=AUTOPILOT
MID = 1500
channels = [MID] * 16

# Arm the craft (CH5 high)
channels[4] = 2000   # ARM
channels[2] = 1100   # Throttle low for arming

print("Arming...")
for _ in range(100):
    send_rc(channels)
    time.sleep(0.02)

# Raise throttle to hover
channels[2] = 1500
print("Hovering...")
for _ in range(200):
    send_rc(channels)
    time.sleep(0.02)

# Enable autopilot mode (CH6 high)
channels[5] = 2000
print("Autopilot engaged!")
while True:
    send_rc(channels)
    time.sleep(0.02)