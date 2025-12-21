---
title: JSON5
tags:
    - python
    - json5
---

JSON5 is an extension of JSON designed to be more human-friendly while staying close to JSON.

**What json5 add compare to json**
- Comments
- multi line string
- hex
- trailing commas 


```bash
pip install json5
```

```python
import json5

with open("config.json5") as f:
    data = json5.load(f)

print(data["name"])   # diffbot

```