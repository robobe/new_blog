---
title: Python zmq basic
tags:
    - python
    - zmq
---

# ZMQ

| Scope        | Transport   | Performance | Complexity | Recommendation |
| ------------ | ----------- | ----------- | ---------- | -------------- |
| [Same process](#inproc) | `inproc://` | ⭐⭐⭐⭐⭐       | ⭐          | ✅ Best         |
| [Same machine](#ipc) | `ipc://`    | ⭐⭐⭐⭐        | ⭐⭐         | ✅ Best         |
| [Network](#tcp)      | `tcp://`    | ⭐⭐⭐         | ⭐⭐         | ✅ Best         |
| Any          | shm         | ⭐⭐⭐⭐        | ⭐⭐⭐⭐⭐      | ❌ Avoid        |


<div class="grid-container">
 <div class="grid-item">
        <a href="pub_sub">
        <p>Pub / Sub</p>
        </a>
    </div>
    <div class="grid-item">
        <a href="request_response">
        <p>Request / Response</p>
        </a>
    </div>
     <div class="grid-item">
        <a href="serialization">
        <p>Serialization</p>
        </a>
    </div>
    
</div>

| Pattern       | Direction | Fan-out | Reliable | Typical use       |
| ------------- | --------- | ------- | -------- | ----------------- |
| REQ/REP       | 1↔1       | ❌       | ✅        | RPC               |
| PUB/SUB       | 1→N       | ✅       | ❌        | Events, streaming |
| PUSH/PULL     | 1→N       | ❌       | ✅        | Work queues       |
| [DEALER/ROUTER](dealer) | N↔N       | ✅       | ✅        | Async services    |
| XPUB/XSUB     | 1→N       | ✅       | ❌        | Brokers           |
| [PAIR](pair)          | 1↔1       | ❌       | ✅        | Simple pipes      |

---

<div class="grid-container">
    <div class="grid-item">
        <a href="zmq_and_shm">
        <p>ZMQ with shm</p>
        </a>
    </div>
        <div class="grid-item">
        <a href="serialization">
        <p>Serialization and Zero copy</p>
        </a>
    </div>
</div>

## inproc
Communication in same process

!!! info "share the same context"
    
```python
import zmq
import threading
import time

ctx = zmq.Context() # type: ignore[attr-defined]

def publisher():
    pub = ctx.socket(zmq.PUB) # type: ignore[attr-defined]
    pub.bind("inproc://events")
    time.sleep(0.1)  # allow subscribers to connect
    pub.send_multipart([b"topic", b"hello"])

def subscriber():
    sub = ctx.socket(zmq.SUB) # type: ignore[attr-defined]
    sub.connect("inproc://events")
    sub.setsockopt(zmq.SUBSCRIBE, b"topic") # type: ignore[attr-defined]
    print(sub.recv_multipart())

t1 = threading.Thread(target=subscriber)
t2 = threading.Thread(target=publisher)

t1.start()
t2.start()

t1.join()
t2.join()

```

---


## ipc

Same machine different process

```python
import zmq
import time
import os
import multiprocessing as mp

SOCKET_PATH = "/tmp/events.ipc"

def publisher():
    # Clean stale socket
    if os.path.exists(SOCKET_PATH):
        os.unlink(SOCKET_PATH)

    ctx = zmq.Context()  # type: ignore[attr-defined]
    pub = ctx.socket(zmq.PUB)  # type: ignore[attr-defined]
    pub.bind(f"ipc://{SOCKET_PATH}")

    time.sleep(0.2)  # allow subscribers to connect
    pub.send_multipart([b"topic", b"hello"])
    time.sleep(0.1)

    pub.close()
    ctx.term()

def subscriber():
    ctx = zmq.Context()  # type: ignore[attr-defined]
    sub = ctx.socket(zmq.SUB)  # type: ignore[attr-defined]

    sub.connect(f"ipc://{SOCKET_PATH}")
    sub.setsockopt(zmq.SUBSCRIBE, b"topic")  # type: ignore[attr-defined]

    print(sub.recv_multipart())

    sub.close()
    ctx.term()

if __name__ == "__main__":
    mp.set_start_method("spawn", force=True)  # safe default

    p_sub = mp.Process(target=subscriber)
    p_pub = mp.Process(target=publisher)

    p_sub.start()
    time.sleep(0.1)   # ensure SUB starts first
    p_pub.start()

    p_pub.join()
    p_sub.join()
```

---

## tcp

```python
import zmq
import time
import multiprocessing as mp

ENDPOINT = "tcp://127.0.0.1:5555"

def publisher():
    ctx = zmq.Context()  # type: ignore[attr-defined]
    pub = ctx.socket(zmq.PUB)  # type: ignore[attr-defined]
    pub.bind(ENDPOINT)

    time.sleep(0.2)  # allow subscribers to connect
    pub.send_multipart([b"topic", b"hello"])
    time.sleep(0.1)

    pub.close()
    ctx.term()

def subscriber():
    ctx = zmq.Context()  # type: ignore[attr-defined]
    sub = ctx.socket(zmq.SUB)  # type: ignore[attr-defined]

    sub.connect(ENDPOINT)
    sub.setsockopt(zmq.SUBSCRIBE, b"topic")  # type: ignore[attr-defined]

    print(sub.recv_multipart())

    sub.close()
    ctx.term()

if __name__ == "__main__":
    mp.set_start_method("spawn", force=True)

    p_sub = mp.Process(target=subscriber)
    p_pub = mp.Process(target=publisher)

    p_sub.start()
    time.sleep(0.1)  # SUB first (important)
    p_pub.start()

    p_pub.join()
    p_sub.join()
```

---

- [zmq_req_rep](zmq_req_rep.md)
- [zmq_req_rep using ipc](zmq_req_rep_ipc.md)
- [pub sub with dataclass and msgpack](pub_sub_msgpack.md)
