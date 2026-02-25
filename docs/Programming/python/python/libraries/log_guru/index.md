---
title: Log Guru
tags:
    - python
    - log
---

Loguru is a modern and very simple logging library for Python.
It replaces the built-in logging module with a much easier API.

```bash
pip install loguru
```

## Demo:
- logging all levels and output all to stdout
- remove the default logger (default config to warning level)

```python
from loguru import logger
import sys

logger.remove()  # remove default handler
logger.add(sys.stdout, level="DEBUG")

logger.debug("Debug message")
logger.info("Info message")
logger.warning("Warning message")
logger.error("Error message")
logger.critical("Critical message")
```

## handlers

Add **file handler**

```python
logger.add("file.log", level="WARNING")
```

Add **rotation file**

keep the last 5 log

```python
logger.add(
    "app.log",
    rotation="5 MB",
    retention=5
)
```

| Feature        | How                  |
| -------------- | -------------------- |
| Rotate by size | `rotation="10 MB"`   |
| Rotate by time | `rotation="1 day"`   |
| Retention      | `retention="7 days"` |
| Compression    | `compression="zip"`  |
| JSON logs      | `serialize=True`     |


!!! tip "rotation pattern"
    There is more pattern check documentation
    