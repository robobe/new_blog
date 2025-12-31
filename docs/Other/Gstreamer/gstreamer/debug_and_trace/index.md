---
title: GStreamer debug and trace
tags:
    - gstreamer
    - debug
    - trace
    - gst-shark
    - identity
---

## Debug
[Basic tutorial 11: Debugging tools](https://gstreamer.freedesktop.org/documentation/tutorials/basic/debugging-tools.html?gi-language=c)
```

```

## Trace
```bash title="from source to sink"
 GST_TRACERS=latency GST_DEBUG=GST_TRACER:7 
```

#TODO: explain
```bash
GST_TRACERS="latency(flags=element+pipeline+reported)" GST_DEBUG=GST_TRACER:7
```

#TODO: gst-shark
[GstShark_](https://developer.ridgerun.com/wiki/index.php/GstShark_-_Getting_Started#Getting_the_code)