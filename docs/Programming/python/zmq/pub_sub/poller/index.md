---
title: ZMQ Pub/Sub with poller
tags:
    - zmq
    - python
    - poller
---

A Poller lets you wait on multiple ZMQ sockets at the same time and tells you which socket is ready (read/write/error).

Poller is useful when you have:
- Multiple ZMQ sockets
- Different socket types
- One thread / one loop


```
PUB A ---> SUB A \
                     ---> application
PUB B ---> SUB B /

```

```python
import zmq

ctx = zmq.Context.instance()

sub_a = ctx.socket(zmq.SUB)
sub_a.setsockopt(zmq.SUBSCRIBE, b"telemetry/")
sub_a.connect("tcp://127.0.0.1:5555")

sub_b = ctx.socket(zmq.SUB)
sub_b.setsockopt(zmq.SUBSCRIBE, b"cam/")
sub_b.connect("tcp://127.0.0.1:5556")

poller = zmq.Poller()
poller.register(sub_a, zmq.POLLIN)
poller.register(sub_b, zmq.POLLIN)

while True:
    events = dict(poller.poll(timeout=1000))  # ms

    if sub_a in events:
        topic, payload = sub_a.recv_multipart()
        print("telemetry:", payload)

    if sub_b in events:
        topic, payload = sub_b.recv_multipart()
        print("camera:", payload)

```

### Demo:

<details>
<summary>Code</summary>
```python
--8<-- "docs/Programming/python/zmq/pub_sub/poller/code/poller_demo.py"
```
</details>