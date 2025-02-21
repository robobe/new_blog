---
tags:
    - gstreamer
    - rtsp
    - server
    - python
---

```
```

```python title="server.py"
--8<-- "docs/Other/Gstreamer/rtsp/server.py"
```

```bash title="player"
gst-launch-1.0 rtspsrc location=rtsp://127.0.0.1:8554/test ! rtph264depay ! avdec_h264 ! autovideosink
```

#todo: explain server
