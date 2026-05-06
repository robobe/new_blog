---
title: ZMQ serialization
tags:
    - zmq
    - msgpack
    - protobuf
    - memoryview
    - zero copy
---

## Common serialization options used with ZeroMQ

| Serialization                    | Speed |  Size | Schema / Versioning        | Cross-language | Zero-copy friendly | Best for                                          | Downsides                                             |
| -------------------------------- | ----: | ----: | -------------------------- | -------------- | ------------------ | ------------------------------------------------- | ----------------------------------------------------- |
| **Raw bytes (no serialization)** | ⭐⭐⭐⭐⭐ | ⭐⭐⭐⭐⭐ | None                       | N/A            | ⭐⭐⭐⭐⭐              | Video frames, audio, tensors, opaque blobs        | You must define your own layout/metadata              |
| **`struct` (binary packing)**    | ⭐⭐⭐⭐⭐ | ⭐⭐⭐⭐⭐ | Manual (you define layout) | ⭐⭐⭐⭐           | ⭐⭐⭐⭐⭐              | Fixed-format headers, tiny fast metadata          | Painful to evolve; endian/alignment concerns          |
| **MessagePack**                  |  ⭐⭐⭐⭐ |  ⭐⭐⭐⭐ | Manual (conventions)       | ⭐⭐⭐⭐           | ⭐⭐⭐                | Fast Python metadata, PUB/SUB payload headers     | No enforced schema; versioning is on you              |
| **Protocol Buffers**             |  ⭐⭐⭐⭐ |  ⭐⭐⭐⭐ | ⭐⭐⭐⭐⭐ (excellent)          | ⭐⭐⭐⭐⭐          | ⭐⭐                 | Stable APIs, multi-language systems               | Compile step, less “dynamic”, not zero-copy           |
| [**FlatBuffers**](#flatbuffer)                  | ⭐⭐⭐⭐⭐ | ⭐⭐⭐⭐⭐ | ⭐⭐⭐⭐                       | ⭐⭐⭐⭐⭐          | ⭐⭐⭐⭐               | High-rate telemetry; fast reads without unpack    | More complex; schema rigidity; tooling learning curve |
| **Cap’n Proto**                  | ⭐⭐⭐⭐⭐ | ⭐⭐⭐⭐⭐ | ⭐⭐⭐⭐                       | ⭐⭐⭐⭐⭐          | ⭐⭐⭐⭐⭐              | Lowest latency; near zero-copy reads              | Ecosystem/tooling is heavier; less common in Python   |
| **JSON**                         |    ⭐⭐ |    ⭐⭐ | Manual                     | ⭐⭐⭐⭐⭐          | ⭐                  | Control plane, debugging, configs                 | Big + slow; numeric/typing ambiguity                  |
| **Pickle (Python only)**         |   ⭐⭐⭐ |   ⭐⭐⭐ | N/A                        | ⭐              | ⭐⭐                 | Quick prototypes within trusted Python-only setup | **Unsafe with untrusted data**; not cross-language    |


---

<div class="grid-container">
 <div class="grid-item">
        <a href="#zero-copy">
        <p>Zero copy</p>
        </a>
    </div>
    <div class="grid-item">
        <a href="#flatbuffer">
        <p>Flat buffers</p>
        </a>
    </div>
     <div class="grid-item">
        <a href="#raw">
        <p>Raw (numpy and memory view)</p>
        </a>
    </div>
    
</div>

## FlatBuffer

With FlatBuffers, the message stays in its binary form and fields are read directly from the buffer.

```bash title="install"
pip install flatbuffers

# install flatc compiler
sudo apt install flatbuffers-compiler
```

### Demo: simple

```title="telemetry.fbs"
namespace Telemetry;

table Telemetry {
  seq:uint32;
  ts:double;
  temp:float;
  msg:string;
}

root_type Telemetry;
```

```bash
flatc --python telemetry.fbs
```


```python title="usage"
import time
import flatbuffers

from Telemetry import Telemetry


def build_telemetry(seq: int, ts: float, temp: float, msg: str) -> bytes:
    b = flatbuffers.Builder(128)

    msg_off = b.CreateString(msg)

    Telemetry.Start(b)
    Telemetry.AddSeq(b, seq)
    Telemetry.AddTs(b, ts)
    Telemetry.AddTemp(b, temp)
    Telemetry.AddMsg(b, msg_off)
    obj = Telemetry.End(b)

    b.Finish(obj)
    return bytes(b.Output())


def read_telemetry(buf: bytes) -> None:
    t = Telemetry.Telemetry.GetRootAsTelemetry(buf, 0)
    print("seq:", t.Seq())
    print("ts :", t.Ts())
    print("temp:", t.Temp())
    print("msg:", t.Msg().decode("utf-8"))


if __name__ == "__main__":
    data = build_telemetry(
        seq=1,
        ts=time.time(),
        temp=36.5,
        msg="h",
    )
    print(len(data))
    read_telemetry(data)

```

---

### Zero-copy

```python title="send"
sock.send(flatbuf_bytes, copy=False)
```

```python title="receiving"
frame = sock.recv(copy=False)
```

### Demo: zmq

<details>
<summary>code</summary>
```
--8<-- "docs/Programming/python/zmq/serialization/code/zmq_flatbuffers.py"
```
</details>

---

## RAW

Raw data = an opaque sequence of bytes.

| Python object          | Valid raw ZMQ data?    |
| ---------------------- | ---------------------- |
| `bytes`                | ✅                      |
| `bytearray`            | ✅                      |
| `memoryview`           | ✅ (best for zero-copy) |
| `numpy.ndarray` buffer | ✅ (via `memoryview`)   |
| `zmq.Frame`            | ✅                      |


```python title=""
# Sender
arr = np.ascontiguousarray(arr)
sock.send_multipart(
    [meta_bytes, memoryview(arr)],
    copy=False
)

```

```python title=""
# Receiver
meta, frame = sock.recv_multipart(copy=False)
arr = np.frombuffer(frame, dtype=...,).reshape(...)

# The NumPy array shares memory with the ZMQ frame.

# That memory is only valid until the next recv or socket close.
```
---

### Demo: Send image with metadata

<details>
<summary>Code</summary>
```
--8<-- "docs/Programming/python/zmq/serialization/code/zmq_image.py"
```
</details>

---

### MemoryView

memoryview is a zero-copy view onto an existing bytes-like buffer.

- It does not allocate a new bytes object.
- It lets you slice/read (and sometimes write) without copying.
- It works with anything that supports the buffer protocol: bytes, bytearray, array, NumPy arrays, and zmq.Frame.

# TODO: code example 