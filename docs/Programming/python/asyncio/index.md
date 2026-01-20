---
title: Python asyncio
tags:
    - asyncio
---

With asyncio, while we wait, the event loop can do other tasks

- asyncio.get_event_loop
- asyncio.sleep
- call_later

```python title="hello world"
import asyncio
import logging
logging.basicConfig(
    format='%(asctime)s - %(levelname)s - %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S',
    level=logging.DEBUG)


async def main():
    logging.debug("Hello ...")
    asyncio.get_event_loop().call_later(
        1.0,
        lambda: logging.debug("Immediate callback executed"))    
    await asyncio.sleep(2)
    logging.debug("... World!")

if __name__ == "__main__":
    asyncio.run(main())
```

The lambda function call after 1 sec event we execute `asyncio.sleep`

```bash
2026-01-20 16:55:01 - DEBUG - Hello ...
2026-01-20 16:55:02 - DEBUG - Immediate callback executed
2026-01-20 16:55:03 - DEBUG - ... World!
```

---

### gather

<details>
<summary>code</summary>
```
--8<-- "docs/Programming/python/asyncio/code/gather.py"
```
</details>

---

### Future

<details>
<summary>future with callback</summary>
```
--8<-- "docs/Programming/python/asyncio/code/future_producer_consumer.py"
```
</details>

<details>
<summary>future with exception</summary>
```
--8<-- "docs/Programming/python/asyncio/code/future_exception.py"
```
</details>

<details>
<summary>future with callback</summary>
```
--8<-- "docs/Programming/python/asyncio/code/future_add_done_callback.py"
```
</details>

<details>
<summary>future with thread safe</summary>
```
--8<-- "docs/Programming/python/asyncio/code/future_callback_thread_safe.py"
```
</details>

---

### Mixing
Mixing Async and Sync: 

<details>
<summary>register corotine in loop from worker thread</summary>
```
--8<-- "docs/Programming/python/asyncio/code/mixing_async_sync.py"
```
</details>

---

### Wait and Timeout

<details>
<summary>simple wait_for</summary>
```
--8<-- "docs/Programming/python/asyncio/code/wait_simple_example.py"
```
</details>

<details>
<summary>wait for the first</summary>
```
--8<-- "docs/Programming/python/asyncio/code/wait_for_the_first_one.py"
```
</details>