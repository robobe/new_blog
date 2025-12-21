---
title: JSON
tags:
    - python
    - json
---

JSON (JavaScript Object Notation) is a lightweight, text-based data format used to store and exchange structured data in a simple, readable way.

- Uses key–value pairs and arrays
- Easy for humans to read and machines to parse
- Commonly used in APIs, configuration files, and data exchange


```json
{
  "name": "robot",
  "enabled": true,
  "count": 3
}

```


```python title="simple.py"
import json
import pprint

try:
    with open("simple.json") as f:
        data = json.load(f)
        pprint.pprint(data)
except json.JSONDecodeError as e:
    print("Invalid JSON:", e)
```

<div class="grid-container">
    <div class="grid-item">
        <a href="json5">
            <p>json5</p>
        </a>
    </div>
    <div class="grid-item">
        <a href="json_schema">
            <p>json schema</p>
        </a>
    </div>
    <!-- <div class="grid-item">
        <a href="toml">
            <p>TOML</p>
        </a>
    </div> -->
</div>