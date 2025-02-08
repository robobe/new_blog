---
tags:
    - zenoh
    - python
---


# Zenoh Python bindings

```bash
pip install eclipse-zenoh
```

---

## Examples

### Pub/Sub
Simple pub sub

```python
import zenoh
import time
import multiprocessing

KEY_EXPRESSION = "demo/example"

def publisher():
    config = zenoh.Config()
    session = zenoh.open(config)
    pub = session.declare_publisher(KEY_EXPRESSION)
    while True:
        pub.put('Hello, Zenoh!')
        time.sleep(1)

def subscriber():
    config = zenoh.Config()
    session = zenoh.open(config)
    sub = session.declare_subscriber(KEY_EXPRESSION, lambda sample: print(f'Received: {sample.payload.to_string()}'))
    while True:
        time.sleep(1)

if __name__ == '__main__':
    pub_process = multiprocessing.Process(target=publisher)
    sub_process = multiprocessing.Process(target=subscriber)

    pub_process.start()
    sub_process.start()

    pub_process.join()
    sub_process.join()
```