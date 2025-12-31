---
title: GStreamer clock
tags:
    - gstreamer
    - clock
    - pts
    - dts
---

The GStreamer clock is the global time reference that the pipeline uses to decide:

**WHEN each buffer should be processed, rendered, or sent**

Without a clock:
- Buffers would flow as fast as possible
- Video would play too fast
- Audio/video would drift
- Network streams would burst

The clock allows:
- Real-time playback
- Audio ↔ video sync
- Smooth network pacing


### Demo: play mp4 file
Change `autovideosink` sync
- **True** (default) use clock to play the video
- **False**: play the video as fast as possible

```bash
gst-launch-1.0 filesrc location=video.mp4 \
! decodebin \
! videoconvert \
! autovideosink sync=false
```

---

### Clock + timestamps (critical pair)

Every buffer may carry a timestamp:

- **PTS** – when to present
- **DTS** – when to decode

The sink compares: buffer PTS  vs  pipeline clock time

Decision:

- PTS > now → wait
- PTS ≈ now → render/send
- PTS < now → late (drop or rush)

!!! tip ""
    This is why sync=true matters.

### Clock source
Only **one** element provides the clock.

Clock provider priority:
- Audio sink (most stable)
- Live source (camera, mic, RTP)
- System clock (fallback)

#### Clock in different pipeline types

##### File playback
- No audio → system clock
- With audio → audio sink clock

###### Demo

```bash title="the output from the above filesrc pipe" linenums="1" hl_lines="10"
Setting pipeline to PAUSED ...
Pipeline is PREROLLING ...
Redistribute latency...
Got context from element 'nvh264dec0': gst.cuda.context=context, gst.cuda.context=(GstCudaContext)"\(GstCudaContext\)\ cudacontext1", cuda-device-id=(uint)0;
Redistribute latency...
Redistribute latency...
Pipeline is PREROLLED ...
Setting pipeline to PLAYING ...
Redistribute latency...
New clock: GstSystemClock
Redistribute latency...
Got EOS from element "pipeline0".
Execution ended after 0:00:00.711912146
Setting pipeline to NULL ...
Freeing pipeline ..
```

#### Camera
- Camera provides clock
- No preroll

```bash
gst-launch-1.0 v4l2src ! videoconvert ! autovideosink

```

#### RTP / UDP streaming
##### Sender
```
appsrc is-live=true do-timestamp=true !
...
udpsink sync=true
```

- Sender timestamps buffers
- Sink uses clock to pace packets

##### Receiver
```
udpsrc !
rtpjitterbuffer !
...
autovideosink
```
    
- rtpjitterbuffer re-aligns timestamps
- Sink syncs to clock

---
### Live vs non-live sources (very important)

| Source        | is-live | Clock behavior    |
| ------------- | ------- | ----------------- |
| filesrc       | false   | preroll, seekable |
| v4l2src       | true    | no preroll        |
| udpsrc        | true    | real-time         |
| appsrc (live) | true    | you control time  |

!!! tip "preroll"
    Preroll is the process where a pipeline buffers just enough data so that playback can start smoothly at time 0.



    