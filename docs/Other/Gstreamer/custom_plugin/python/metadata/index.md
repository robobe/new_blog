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
| GstCustomMeta              | ✅ Very Good  | ❌       | ⚠ Limited GI support |
| Bus Message (GstStructure) | ✅            | ❌       | ✅ Easy               |



---

## GstStructure

<details>
<summary>plugin</summary>
```
--8<-- "docs/Other/Gstreamer/custom_plugin/python/metadata/code/plugins/simplemeta.py"
```
</details>


<details>
<summary>usage</summary>
```
--8<-- "docs/Other/Gstreamer/custom_plugin/python/metadata/code/simplemeta.py"
```
</details>

---

## GstCustomMeta

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