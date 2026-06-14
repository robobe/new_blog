---
title: Plugin metadata
tags:
    - GstMeta
    - GstCustomMeta
    - GstStructure
---

| Method                     | Same Process | Network | Python Friendly      |
| -------------------------- | ------------ | ------- | -------------------- |
| Custom GstMeta  (c or rust)           | ✅ Best       | ❌       | ⚠ Requires binding   |
| [GstCustomMeta](#gstcustommeta)              | ✅ Very Good  | ❌       | ⚠ Limited GI support |
| [Bus Message (GstStructure)](#gststructure) | ✅            | ❌       | ✅ Easy               |



---

## GstStructure

<details>
<summary>plugin</summary>
```
--8<-- "docs/Other/Gstreamer/custom_plugin/python/metadata/code/plugins/simplemeta.py"
```
</details>

The `simplemeta` element posts a `Gst.MessageType.APPLICATION` message on the pipeline bus for each buffer.
The message carries a `GstStructure` named `tracker` with the metadata fields `x` and `y`.
The application reads messages from the bus, filters application messages, extracts the structure, and prints those values.

<details>
<summary>usage</summary>
```
--8<-- "docs/Other/Gstreamer/custom_plugin/python/metadata/code/simplemeta.py"
```
</details>

---

## GstCustomMeta

`GstMeta` is the base mechanism GStreamer uses to attach metadata directly to a
`Gst.Buffer`. A custom `GstMeta` implementation defines its own metadata type,
allocation logic, transform/copy behavior, and usually needs C or Rust code plus
language bindings before Python can use it comfortably.

`GstCustomMeta` is a built-in `GstMeta` implementation whose payload is a named
`Gst.Structure`. Instead of defining a new native metadata type, the plugin
registers a custom metadata name and stores fields in that structure. This makes
it useful for small, structured values that need to travel with the buffer inside
the same process, while avoiding most of the boilerplate required by a full
custom `GstMeta`.

The tradeoff is that `GstCustomMeta` is less flexible than a native custom
`GstMeta`: the data model is limited to `Gst.Structure` fields, transform
behavior is generic, and Python GI access can be limited depending on the
GStreamer version and bindings.

<details>
<summary>plugin</summary>
```
--8<-- "docs/Other/Gstreamer/custom_plugin/python/metadata/code/plugins/simplegstmeta.py"
```
</details>

<details>
<summary>usage</summary>
```
--8<-- "docs/Other/Gstreamer/custom_plugin/python/metadata/code/simplegstmeta_appsink_loop.py"
```
</details>

### Demo
- Add another plugin that use the data

<details>
<summary>Plugin</summary>
```
--8<-- "docs/Other/Gstreamer/custom_plugin/python/metadata/code/plugins/usegstmeta.py"
```
</details>

<details>
<summary>usage</summary>
```
--8<-- "docs/Other/Gstreamer/custom_plugin/python/metadata/code/simplegstmeta_downstream_plugin.py"
```
</details>

!!! info ""
    the python binding get the metadata from the **usegstmeta** plugin in that resend the data from **simplegstmeta**
