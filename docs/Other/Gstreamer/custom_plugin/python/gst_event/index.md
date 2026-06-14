---
title: Gst.Event
tags:
    - gstreamer
    - python
    - Gst.Event
    - custom event
---

# Gst.Event

`Gst.Event` is a control message that travels through the pipeline pads.
Buffers carry media data; events carry pipeline control information such as
`EOS`, `CAPS`, `SEGMENT`, `FLUSH_START`, or application-defined commands.

The main idea is direction:

| Event direction | Flow | Common use |
| --------------- | ---- | ---------- |
| Downstream | source to sink | caps, segment, eos, custom app command |
| Upstream | sink to source | seek, navigation, latency query result path |

In a Python plugin, a simple way to listen for downstream events is to inherit
from `GstBase.BaseTransform` and override `do_sink_event()`. The plugin checks
the event type, reads the attached `GstStructure`, handles the fields it knows,
then calls the parent implementation so normal GStreamer events still continue
through the element.

## Minimal Plugin

This element listens for a custom downstream event named `app-control`.

<details>
<summary>plugin</summary>

```python
--8<-- "docs/Other/Gstreamer/custom_plugin/python/gst_event/code/plugins/eventlistener.py"
```

</details>

## Minimal Application

The application creates a normal pipeline, finds the plugin by name, builds a
custom downstream event, and sends it to the plugin sink pad.

<details>
<summary>usage</summary>

```python
--8<-- "docs/Other/Gstreamer/custom_plugin/python/gst_event/code/send_event.py"
```

</details>

For a quick test, put the plugin file under `python/eventlistener.py`, then run
the application with `GST_PLUGIN_PATH` pointing at the directory that contains
that `python/` folder:

```bash
GST_PLUGIN_PATH=$PWD python3 send_event.py
```

Expected plugin output:

```text
event: set-threshold 42
```
