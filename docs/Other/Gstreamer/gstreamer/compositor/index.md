---
Title: GStreamer compositor
tags:
    - gstreamer
    - compositor
---

{{ page_folder_links() }}

```bash title="simple pipe with compositor"
--8<-- "docs/Other/Gstreamer/gstreamer/compositor/code/simple_compositor.sh"
```

---

## Dynamic control compositor

```bash title="simple pipe with compositor"
--8<-- "docs/Other/Gstreamer/gstreamer/compositor/code/dynamic_control_compasitor.py"
```

### usage
```bash
# switch to other layout
curl -X 'GET' \
  'http://localhost:8000/layout/side_by_side' \
  -H 'accept: application/json'

# switch to other layout
curl -X 'GET' \
  'http://localhost:8000/layout/pip' \
  -H 'accept: application/json'
```