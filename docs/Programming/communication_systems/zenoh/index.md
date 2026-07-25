---
title: Zenoh distributed communication
tags:
    - programming
    - communication
    - distributed-systems
    - zenoh
    - python
    - pub-sub
---

# Zenoh distributed communication

Zenoh is a distributed communication system for moving data between
applications.

<div class="grid-container">
    <div class="grid-item">
        <a href="config">
            <p>Zenoh Config</p>
        </a>
    </div>
    <div class="grid-item">
        <a href="cpp">
            <p>Zenoh C++</p>
        </a>
    </div>
</div>

It is useful when you need pub/sub, queries, storage, and communication across
different machines or networks.

You can think about Zenoh as a data-centric communication layer:

- publishers write values to keys
- subscribers receive updates for keys they are interested in
- queries can ask for data
- storages can keep the latest values
- routers can connect distributed nodes

Zenoh is often interesting for robotics, IoT, edge systems, distributed
applications, and systems that need efficient communication without building a
custom protocol from zero.

## Core idea

Zenoh uses key expressions.

A key looks like a path:

```text
robot/imu/data
robot/camera/status
factory/line1/temp
```

A publisher writes data to a key.

A subscriber listens to a key or key pattern.

Example pattern:

```text
robot/**
```

This can match many keys under `robot/`.

## Install Python package

Install the Python package:

```bash
pip install eclipse-zenoh
```

## Minimal pub/sub in one Python application

This example runs a subscriber process and a publisher process from one Python
file.

Create `pub_sub_demo.py`:

```python
import multiprocessing
import time

import zenoh


def run_publisher():
    key = "demo/robot/counter"

    with zenoh.open(zenoh.Config()) as session:
        publisher = session.declare_publisher(key)

        for counter in range(5):
            message = f"counter={counter}"
            print(f"publish {key}: {message}")
            publisher.put(message)
            time.sleep(1)


def listener(sample):
    payload = sample.payload.to_string()
    print(f"received {sample.key_expr}: {payload}")


def run_subscriber():
    key_expression = "demo/robot/**"

    with zenoh.open(zenoh.Config()) as session:
        session.declare_subscriber(key_expression, listener)
        print(f"subscribed to {key_expression}")
        time.sleep(8)


def main():
    subscriber_process = multiprocessing.Process(target=run_subscriber)
    publisher_process = multiprocessing.Process(target=run_publisher)

    subscriber_process.start()

    # Give the subscriber time to declare its subscription before publishing.
    time.sleep(1)

    publisher_process.start()

    publisher_process.join()
    subscriber_process.join()


if __name__ == "__main__":
    main()
```

Run:

```bash
python3 pub_sub_demo.py
```

Expected output:

```text
subscribed to demo/robot/**
publish demo/robot/counter: counter=0
received demo/robot/counter: counter=0
publish demo/robot/counter: counter=1
received demo/robot/counter: counter=1
publish demo/robot/counter: counter=2
received demo/robot/counter: counter=2
```

## What happens

The publisher opens a Zenoh session and declares that it will publish on:

```text
demo/robot/counter
```

The subscriber opens a Zenoh session and subscribes to:

```text
demo/robot/**
```

Because `demo/robot/counter` matches `demo/robot/**`, the subscriber receives
the published samples.

The example uses two processes because it behaves like two independent
applications, but still lets you run the demo from one command.

The payload is received as Zenoh bytes. In Python, use:

```python
sample.payload.to_string()
```

to convert the payload to a string.

## When to learn Zenoh

Learn Zenoh when your project needs:

- pub/sub communication
- distributed nodes
- machine-to-machine data exchange
- robot or IoT telemetry
- data routing between networks
- queries over distributed data
- storage of latest values

## Topics to learn next

Important Zenoh topics:

- key expressions and wildcards
- publishers and subscribers
- query and queryable
- routers
- peer-to-peer vs routed deployment
- configuration files
- transports
- access control and authentication
- storage plugins
- bridging Zenoh with other systems

## Reference

- [Zenoh docs](https://zenoh.io/docs/)
- [Zenoh first Python app](https://zenoh.io/docs/getting-started/first-app/)
- [Zenoh abstractions](https://zenoh.io/docs/manual/abstractions/)
