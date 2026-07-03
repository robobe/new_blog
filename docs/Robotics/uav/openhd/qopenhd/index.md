---
title: QOpenHD
tags:
    - uav
    - fpv
    - openhd
    - qopenhd
---

QOpenHD is the default Qt companion application for OpenHD ground stations. It
shows the live video stream, draws the OSD, displays telemetry, and lets the
operator change OpenHD settings from a ground computer or device connected to
the OpenHD ground unit.

QOpenHD is not the OpenHD radio/link service itself. It is the user interface
that talks to a running OpenHD system over MAVLink and receives the video stream.

Source:
[OpenHD/QOpenHD 2.7-evo](https://github.com/OpenHD/QOpenHD/tree/2.7-evo)

## Install on Linux

```bash
 git clone --recurse-submodules https://github.com/OpenHD/QOpenHD.git
 cd QOpenHD/
 mkdir build
 cd build/
 qmake ..
 cd ..
 sudo bash install_build_dep.sh 
 ./build_qmake.sh 
 # Run
 ./build/release/QOpenHD 
```


### Stream Test video

```bash
gst-launch-1.0 -v videotestsrc is-live=true pattern=smpte   \
    ! video/x-raw,width=640,height=480,framerate=30/1   \
    ! videoconvert   \
    ! videoscale   ! video/x-raw,format=I420,width=640,height=480,framerate=30/1   \
    ! x264enc bitrate=1500 \
        tune=zerolatency \
        speed-preset=ultrafast \
        key-int-max=30 \
        bframes=0 \
        byte-stream=true \
        aud=true \
        intra-refresh=false \
        sliced-threads=false \
        threads=1   \
    ! video/x-h264,stream-format=byte-stream,alignment=au,profile=constrained-baseline  \
    ! h264parse config-interval=1 \
    ! rtph264pay pt=96 mtu=800 config-interval=1 aggregate-mode=zero-latency \
    ! udpsink host=127.0.0.1 port=5600 sync=false async=false
```

```bash
gst-launch-1.0 videotestsrc is-live=true pattern=smpte \
    ! video/x-raw,format=I420,width=640,height=480,framerate=30/1 \
    ! x264enc bitrate=1500 \
        speed-preset=ultrafast \
        tune=zerolatency \
        key-int-max=10 \
        intra-refresh=false \
        sliced-threads=false \
        threads=1 \
    ! h264parse config-interval=-1 \
    ! rtph264pay pt=96 mtu=1024 config-interval=1 \
    ! udpsink host=127.0.0.1 port=5600 sync=false async=false
```


### Sim mavlink

<details>
<summary>Mavlink code</summary>

```python
#!/usr/bin/env python3
import time, math
from pymavlink import mavutil

mav = mavutil.mavlink_connection(
    "udpout:127.0.0.1:14550",
    source_system=1,
    source_component=mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1,
)

lat = int(32.0853 * 1e7)
lon = int(34.7818 * 1e7)
home_sent = False
t0 = time.time()

while True:
    t = time.time() - t0
    boot_ms = int(t * 1000)

    mav.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_QUADROTOR,
        mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        0,
        mavutil.mavlink.MAV_STATE_ACTIVE,
    )

    mav.mav.sys_status_send(
        0, 0, 0, 500,
        16000,      # battery mV
        1200,       # current cA = 12A
        82,         # battery %
        0, 0, 0, 0, 0, 0,
    )

    mav.mav.gps_raw_int_send(
        int(time.time() * 1e6),
        3,          # 3D fix
        lat,
        lon,
        120000,     # altitude mm
        80, 120,    # hdop/vdop * 100
        500,        # velocity cm/s
        9000,       # course deg * 100
        14,         # satellites
    )

    mav.mav.global_position_int_send(
        boot_ms,
        lat + int(math.sin(t / 20) * 1000),
        lon + int(math.cos(t / 20) * 1000),
        120000,     # MSL alt mm
        50000,      # relative alt mm
        500, 0, 0,  # vx/vy/vz cm/s
        9000,       # heading deg * 100
    )

    mav.mav.attitude_send(
        boot_ms,
        math.radians(10 * math.sin(t)),
        math.radians(5 * math.cos(t)),
        math.radians(90),
        0, 0, 0,
    )

    mav.mav.vfr_hud_send(
        12.0,       # airspeed
        10.0,       # groundspeed
        90,         # heading
        45,         # throttle
        120.0,      # alt
        0.4,        # climb
    )

    if not home_sent:
        mav.mav.home_position_send(
            lat, lon, 120000,
            0, 0, 0,
            [1, 0, 0, 0],
            0, 0, 0,
        )
        home_sent = True

    time.sleep(0.1)
```
</details>

