#!/usr/bin/env python3
"""
Minimal ZMQ PUB/SUB image demo
Metadata: frame_id only (struct-packed uint32)
Message layout:
  [ topic ][ meta(frame_id) ][ payload(raw image bytes) ]
"""
import time
import struct
import multiprocessing as mp

import numpy as np
import zmq


# ----------------------------
# Metadata (just frame_id)
# ----------------------------
# little-endian uint32
META_FMT = "<I"
META_SIZE = struct.calcsize(META_FMT)


def pack_meta(frame_id: int) -> bytes:
    return struct.pack(META_FMT, frame_id)


def unpack_meta(b: bytes) -> int:
    if len(b) != META_SIZE:
        raise ValueError("bad meta size")
    (frame_id,) = struct.unpack(META_FMT, b)
    return frame_id


# ----------------------------
# Publisher
# ----------------------------
def publisher_proc(bind_ep: str, topic: bytes = b"cam/", fps: float = 5.0) -> None:
    ctx = zmq.Context.instance()
    pub = ctx.socket(zmq.PUB)
    pub.bind(bind_ep)

    # Slow joiner workaround
    time.sleep(0.8)

    frame_id = 0
    period = 1.0 / fps

    # Example image: 240x320 grayscale uint8
    h, w = 240, 320

    while True:
        img = np.random.randint(0, 256, size=(h, w), dtype=np.uint8)
        img = np.ascontiguousarray(img)

        meta = pack_meta(frame_id)

        # Multipart: [topic][meta][payload]
        pub.send_multipart(
            [topic, meta, memoryview(img)],
            copy=False,
        )

        frame_id += 1
        time.sleep(period)


# ----------------------------
# Subscriber
# ----------------------------
def subscriber_proc(connect_ep: str, subscribe_prefix: bytes = b"cam/") -> None:
    ctx = zmq.Context.instance()
    sub = ctx.socket(zmq.SUB)
    sub.setsockopt(zmq.SUBSCRIBE, subscribe_prefix)
    sub.connect(connect_ep)

    print("SUB connected to:", connect_ep)

    while True:
        topic_f, meta_f, payload_f = sub.recv_multipart(copy=False)

        frame_id = unpack_meta(bytes(meta_f))

        # Zero-copy view of payload
        buf = memoryview(payload_f)

        # Reconstruct image (we must know shape/dtype out-of-band here)
        img = np.frombuffer(buf, dtype=np.uint8).reshape((240, 320))

        print(f"topic={bytes(topic_f).decode("utf-8")} frame_id={frame_id} img.shape={img.shape}")


# ----------------------------
# Main (multiprocessing)
# ----------------------------
def main() -> None:
    mp.set_start_method("spawn", force=True)

    ep = "tcp://127.0.0.1:5555"

    pub_p = mp.Process(target=publisher_proc, args=(ep,), daemon=True)
    sub_p = mp.Process(target=subscriber_proc, args=(ep,), daemon=True)

    pub_p.start()
    sub_p.start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        for p in (sub_p, pub_p):
            if p.is_alive():
                p.terminate()
        for p in (sub_p, pub_p):
            p.join(timeout=1.0)


if __name__ == "__main__":
    main()