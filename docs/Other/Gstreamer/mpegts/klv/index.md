---
tags:
    - klv
    - gstreamer
---

# KLV

A KLV stream is a sequence of Key-Length-Value encoded messages
Using gstreamer we can send and receive klv data in `SMPTE 336M` standard

!!! note "SMPTE 336M"
    The standard require 16 byte Universal Label (UL) as the key. 
    [more](https://www.ietf.org/archive/id/draft-ietf-avt-rtp-klv-01.html)

     

## Demo
Send and Receive klv using gstreamer python binding

```python title="sender"
--8<-- "docs/Other/Gstreamer/mpegts/klv/code/klv_sender.py"
```

```python title="receiver"
--8<-- "docs/Other/Gstreamer/mpegts/klv/code/klv_receiver.py"
```