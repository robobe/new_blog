#!/usr/bin/env python3
"""
Multiple PUB processes + one SUB process using zmq.Poller.

Messages are multipart:
  [topic][publisher_name][payload]

Subscriber:
- creates ONE SUB socket per publisher endpoint
- subscribes to selected topic prefixes
- uses Poller to read whichever socket becomes ready

Run:
  python3 poller_pubsub_multiprocess.py
Stop:
  Ctrl+C
"""
import time
import multiprocessing as mp
from typing import Iterable

import zmq


def publisher_proc(pub_name: str, bind_endpoint: str, topics: Iterable[str], interval_s: float = 0.25) -> None:
    ctx = zmq.Context.instance()
    pub = ctx.socket(zmq.PUB)
    pub.bind(bind_endpoint)

    # Slow-joiner workaround
    time.sleep(0.8)

    topics_b = [t.encode("utf-8") for t in topics]
    seq = 0

    while True:
        now = time.time()
        for topic in topics_b:
            payload = f"seq={seq} ts={now:.3f}".encode("utf-8")
            pub.send_multipart([topic, pub_name.encode("utf-8"), payload])
        seq += 1
        time.sleep(interval_s)


def subscriber_poller_proc(connect_endpoints: list[str], subscribe_prefixes: Iterable[str]) -> None:
    ctx = zmq.Context.instance()
    poller = zmq.Poller()

    subs: list[zmq.Socket] = []

    # One SUB socket per endpoint (so Poller has multiple sockets to watch)
    for ep in connect_endpoints:
        s = ctx.socket(zmq.SUB)

        # Set subscriptions BEFORE connect
        for prefix in subscribe_prefixes:
            s.setsockopt(zmq.SUBSCRIBE, prefix.encode("utf-8"))

        s.connect(ep)
        poller.register(s, zmq.POLLIN)
        subs.append(s)

    print("SUB sockets connected to:")
    for ep in connect_endpoints:
        print("  ", ep)
    print("Subscribed prefixes:")
    for p in subscribe_prefixes:
        print("  ", p)

    while True:
        # Wait up to 1000ms for any socket to become readable
        # events = dict(poller.poll())   # ← blocking
        events = dict(poller.poll(timeout=1000))

        if not events:
            # No messages this second (optional)
            continue

        # Handle all ready sockets (could be >1)
        for s in events.keys():
            # Drain quickly in case multiple messages queued
            while True:
                try:
                    topic, pub_name, payload = s.recv_multipart(flags=zmq.DONTWAIT)
                except zmq.Again:
                    break
                print(f"[{topic.decode()}] from={pub_name.decode()} payload={payload.decode()}")


def main() -> None:
    mp.set_start_method("spawn", force=True)

    publishers = [
        {
            "name": "pubA",
            "bind": "tcp://127.0.0.1:5555",
            "topics": ["telemetry/", "imu/", "debug/"],
        },
        {
            "name": "pubB",
            "bind": "tcp://127.0.0.1:5556",
            "topics": ["telemetry/", "cam/", "status/"],
        },
        {
            "name": "pubC",
            "bind": "tcp://127.0.0.1:5557",
            "topics": ["cam/", "debug/", "gps/"],
        },
    ]

    # Subscriber only wants these topic prefixes:
    subscribe_prefixes = ["telemetry/", "cam/"]

    pub_procs: list[mp.Process] = []
    sub_proc: mp.Process | None = None

    try:
        # Start publishers
        for p in publishers:
            proc = mp.Process(
                target=publisher_proc,
                args=(p["name"], p["bind"], p["topics"]),
                daemon=True,
            )
            proc.start()
            pub_procs.append(proc)

        # Start subscriber (poller)
        sub_proc = mp.Process(
            target=subscriber_poller_proc,
            args=([p["bind"] for p in publishers], subscribe_prefixes),
            daemon=True,
        )
        sub_proc.start()

        # Keep main process alive
        while True:
            time.sleep(1)

    except KeyboardInterrupt:
        print("\nStopping...")

    finally:
        if sub_proc is not None and sub_proc.is_alive():
            sub_proc.terminate()
            sub_proc.join(timeout=1.0)

        for proc in pub_procs:
            if proc.is_alive():
                proc.terminate()
        for proc in pub_procs:
            proc.join(timeout=1.0)


if __name__ == "__main__":
    main()
