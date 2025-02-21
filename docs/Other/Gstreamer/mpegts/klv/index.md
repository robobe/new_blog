---
tags:
    - gstreamer
    - mpegts
    - klv
    - metadata
    - python
---


<details><summary>mpeg send image and klv data</summary>
```python title="sender"
--8<-- "docs/Other/Gstreamer/mpegts/klv/appsrc_mpeg_klv_udp.py"
```
</details>

<details><summary>mpeg receiver parse klv data</summary>
```python title="receiver"
--8<-- "docs/Other/Gstreamer/mpegts/klv/play_mpeg_video_parse_klv.py"
```
</details>


![alt text](images/output.png)