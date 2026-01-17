---
title: ZMQ Req / Rep pattern
tags:
    - zmq
    - python
    - request
    - replay
---

REQ/REP is ZeroMQ’s synchronous request–reply pattern. like **RPC**

- REQ (client) → sends a request
- REP (server) → processes it and sends exactly one reply


# Demo: Client/Server Req/Rep zmq pattern with msgpack

Synchronous messaging pattern where one side (the Requester) sends a request, and the other side (the Replier) sends a response.

- socket zmq.REQ will block on send unless it has successfully received a reply back.
- socket zmq.REP will block on recv unless it has received a request.

!!! tip timeout
    If you want to set a timeout on the REQ socket, you can use the `zmq.POLLIN` flag with `zmq.Poller` to check if there is a message to receive. or use RCVTIMEO on the socket.
     

### install
```bash title="install dependencies"
pip install msgpack
pip install pyzmq
```

## Demo: Req/Rep zmq pattern with msgpack
ZMQ Req/Rep zmq pattern with python dataclasses and msgpack




<details>
    <summary>code</summary>

```python title="req/rep" linenums="1" hl_lines="28 42"
import multiprocessing
import logging
from dataclasses import dataclass, asdict
import msgpack
import zmq

FMT = "%(asctime)s - %(lineno)s - %(levelname)s - %(message)s"
logging.basicConfig(format=FMT, level=logging.INFO)

log = logging.getLogger(__name__)

TOPIC = b"topic"
SERVICE_PORT = 5555


@dataclass
class Data_Request:
    f_int: int
    f_float: float
    f_string: str


@dataclass
class Data_Response:
    success: bool


def server():
    context = zmq.Context()
    socket = context.socket(zmq.REP)
    socket.bind(f"tcp://*:{SERVICE_PORT}")
    topic, data = socket.recv_multipart()

    msg = msgpack.unpackb(data)
    log.info(f"server get request: {msg}")

    response = Data_Response(success=True)
    data = msgpack.packb(asdict(response))
    socket.send(data)


def client():
    context = zmq.Context()
    socket = context.socket(zmq.REQ)
    socket.connect(f"tcp://127.0.0.1:{SERVICE_PORT}")

    # Create request msg
    msg = Data_Request(1, 2.0, "string")
    raw = asdict(msg)
    data = msgpack.packb(raw)
    socket.send_multipart([TOPIC, data])

    # Recv response from server
    data = socket.recv()
    # unpack socket data
    raw = msgpack.unpackb(data)
    # Convert to msg
    msg = Data_Response(**raw)
    log.info(f"server response: {msg.success}")


if __name__ == "__main__":
    p_server = multiprocessing.Process(target=server)
    p_client = multiprocessing.Process(target=client)
    p_client.start()
    p_server.start()

    p_server.join()
    p_client.join()

```
</details>





---

## Demo2 : Using socket RCVTIMEO

<details>
    <summary>code</summary>

```python
--8<-- "docs/Programming/python/zmq/request_response/code/rcvtimeo.py"
```
</details>



---

## Demo3 : Using zmq.Poller
<details>
    <summary>code</summary>

```python
--8<-- "docs/Programming/python/zmq/request_response/code/poller.py"
```
</details>