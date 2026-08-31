---
title: ZMQ cpp learn path
tags:
    - zmq
    - cpp
---

ZeroMQ is a messaging library for connecting parts of an application. Follow
the stages in order: begin with one client and server, then add data formats,
other messaging patterns, reliability, and concurrency.

| Stage | Topic | Exercise |
| --- | --- | --- |
| 1 | [Context, sockets, bind/connect, and REQ/REP](request_reply/index.md) | Hello request-reply |
| 2 | [Messages and multipart frames](messages/index.md) | Send text and binary bytes |
| 3 | [MessagePack structs and OpenCV images](serialization/index.md) | Send a structured image |
| 4     | Buffer ownership and zero-copy | Send an existing buffer    |
| 5     | PUB/SUB                        | telemetry publisher        |
| 6     | PUSH/PULL                      | processing pipeline        |
| 7     | Polling                        | multiple sockets           |
| 8     | Timeouts/errors                | robust communication       |
| 9     | Multipart messages             | topic + payload            |
| 10    | HWM/backpressure               | slow subscriber experiment |
| 11    | Threading rules                | worker architecture        |
| 12    | Project                        | MAVLink + ZMQ              |

## Installation

```bash
sudo apt update
sudo apt install libzmq3-dev cppzmq-dev libmsgpack-dev libopencv-dev
```

The optional Python interoperability example also needs:

```bash
python3 -m pip install pyzmq msgpack numpy
```
