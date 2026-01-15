#!/usr/bin/env python3
"""
Multiple PUB processes (each publishes on multiple topics),
one SUB process (subscribes only to specific topic prefixes).

Run:
  python3 pubsub_multiprocess_topics.py

Stop with Ctrl+C.
"""
import time
import multiprocessing as mp
from typing import Iterable

import zmq


def publisher_proc(pub_name: str, bind_endpoint: str, topics: Iterable[str], interval_s: float = 0.25) -> None:
    """
    Publish multipart messages:
      [topic][publisher_name][payload]
    """
    ctx = zmq.Context.instance()
    pub = ctx.socket(zmq.PUB)
    pub.bind(bind_endpoint)

    # Slow-joiner workaround: give SUB time to connect + subscribe
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


def subscriber_proc(connect_endpoints: list[str], subscribe_topics: Iterable[str]) -> None:
    """
    Subscribe only to the given topic prefixes.
    """
    ctx = zmq.Context.instance()
    sub = ctx.socket(zmq.SUB)

    # Important: set subscriptions BEFORE connecting (helps avoid missing early messages)
    for t in subscribe_topics:
        sub.setsockopt(zmq.SUBSCRIBE, t.encode("utf-8"))

    for ep in connect_endpoints:
        sub.connect(ep)

    print("SUB connected to:")
    for ep in connect_endpoints:
        print("  ", ep)
    print("SUB subscriptions:")
    for t in subscribe_topics:
        print("  ", t)

    while True:
        topic, pub_name, payload = sub.recv_multipart()
        print(f"[{topic.decode()}] from={pub_name.decode()} payload={payload.decode()}")


def main() -> None:
    mp.set_start_method("spawn", force=True)

    # Two publishers, each with multiple topics
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
    ]

    # Subscriber wants only these topic prefixes
    subscribe_topics = ["telemetry/", "cam/"]  # prefix match

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

        # Start subscriber (as its own process too, so everything is multiprocess)
        sub_proc = mp.Process(
            target=subscriber_proc,
            args=([p["bind"] for p in publishers], subscribe_topics),
            daemon=True,
        )
        sub_proc.start()

        # Keep main alive
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
