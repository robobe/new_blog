---
tags:
    - klv
---

# KLV

A KLV stream is a sequence of Key-Length-Value encoded messages

<div class="grid-container">
    <div class="grid-item">
        <a href="klv_stanag">
        <p>MISB/STANAG</p>
        </a>
    </div>
    <div class="grid-item">
    <a href="">
        <p>TBD</p>
        </a>
    </div>
    <div class="grid-item">
        <a href="">
        <p>TBD</p>
        </a>
    </div>
    
</div>

|   | Description  |
|---|---|
| MISB  | Motion Imagery Standard Board  |
| ST 601 | MISB Defines UAS Datalink Local Set (core telemetry format) |
| STANAG 4609 | NATO Standardization Agreement (include MISB ST 601) |
| UAS metadata | Sensor telemetry sent with drone imagery |

### Unmanned Air System (UAS) Metadata
UAS metadata refers to the telemetry and context information transmitted alongside video or sensor feeds from an Unmanned Aerial System (drone/UAV). It includes:

- Timestamp
- Platform orientation
- Platform position
- more ..

---

## Demo: using klvdata python library
[klvdata github](https://github.com/paretech/klvdata/blob/master/README.rst)

```bash
pip install klvdata
```

---

## Reference
- [MISB ST 0601.8 UAS Datalink Local Set](https://upload.wikimedia.org/wikipedia/commons/1/19/MISB_Standard_0601.pdf)
- [mpg video with klv stream](http://samples.ffmpeg.org/MPEG2/mpegts-klv/Day%20Flight.mpg)
- [klvdata - python library for MISB 601 ](https://github.com/paretech/klvdata/blob/master/README.rst)