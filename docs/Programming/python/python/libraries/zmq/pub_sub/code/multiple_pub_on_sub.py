#!/usr/bin/env python3
import time
import multiprocessing as mp
import zmq


def publisher_proc(pub_id: int, endpoint: str, interval_s: float = 0.2) -> None:
    ctx = zmq.Context.instance()
    pub = ctx.socket(zmq.PUB)
    pub.bind(endpoint)

    

    seq = 0
    topic = f"pub{pub_id}".encode()

    while True:
        payload = f"hello from pub{pub_id} seq={seq} ts={time.time():.3f}".encode()
        pub.send_multipart([topic, payload])
        seq += 1
        time.sleep(interval_s)


def subscriber_blocking(endpoints: list[str], subscribe_prefix: bytes = b"") -> None:
    ctx = zmq.Context.instance()
    sub = ctx.socket(zmq.SUB)

    # Subscribe to everything (b"") or a prefix like b"pub2"
    sub.setsockopt(zmq.SUBSCRIBE, subscribe_prefix)

    for ep in endpoints:
        sub.connect(ep)

    print(f"SUB connected to: {endpoints}")
    print(f"SUB filter prefix: {subscribe_prefix!r}")

    while True:
        topic, payload = sub.recv_multipart()  # blocking
        print(f"[{topic.decode()}] {payload.decode()}")


def main() -> None:

    endpoints = [
        "tcp://127.0.0.1:5555",
        "tcp://127.0.0.1:5556",
        "tcp://127.0.0.1:5557",
    ]

    procs: list[mp.Process] = []
    try:
        # Start publishers
        for i, ep in enumerate(endpoints, start=1):
            p = mp.Process(target=publisher_proc, args=(i, ep), daemon=True)
            p.start()
            procs.append(p)

        time.sleep(1.0)  # wait for publishers to start
        # Run the subscriber (blocking)
        subscriber_blocking(endpoints, subscribe_prefix=b"")

    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        for p in procs:
            if p.is_alive():
                p.terminate()
        for p in procs:
            p.join(timeout=1.0)


if __name__ == "__main__":
    main()