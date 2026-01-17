#!/usr/bin/env python3
import time
import multiprocessing as mp
from typing import Iterable

import zmq
import flatbuffers
from flatbuffers.table import Table
from flatbuffers import packer, encode, number_types
from Telemetry import Telemetry  # type: ignore
# ============================================================
# Minimal inline "generated" FlatBuffers code for:
#
# table Telemetry {
#   seq:uint32;
#   ts:double;
#   temp:float;
#   msg:string;
# }
# root_type Telemetry;
# ============================================================


def build_telemetry_flatbuf(seq: int, ts: float, temp: float, msg: str) -> bytes:
    """
    Build a Telemetry FlatBuffer and return the underlying bytes.
    """
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


# ============================================================
# ZMQ PUB/SUB multiprocessing demo
# ============================================================

def publisher_proc(pub_name: str, bind_endpoint: str, topics: Iterable[str], interval_s: float = 0.25) -> None:
    ctx = zmq.Context.instance()
    pub = ctx.socket(zmq.PUB)
    pub.bind(bind_endpoint)

    # Slow-joiner workaround: give subscriber time to connect + set SUBSCRIBE
    time.sleep(0.8)

    seq = 0
    topics_b = [t.encode("utf-8") for t in topics]

    while True:
        now = time.time()
        for topic in topics_b:
            msg = build_telemetry_flatbuf(
                seq=seq,
                ts=now,
                temp=20.0 + (seq % 10) * 0.5,
                msg=f"from {pub_name} on {topic.decode()}",
            )
            # multipart: [topic][flatbuffer_bytes]
            pub.send_multipart([topic, msg], copy=False)
        seq += 1
        time.sleep(interval_s)


def subscriber_proc(connect_endpoints: list[str], subscribe_prefixes: Iterable[str]) -> None:
    ctx = zmq.Context.instance()
    sub = ctx.socket(zmq.SUB)

    # Subscribe to selected topic prefixes (prefix match!)
    for p in subscribe_prefixes:
        sub.setsockopt(zmq.SUBSCRIBE, p.encode("utf-8"))

    for ep in connect_endpoints:
        sub.connect(ep)

    print("SUB connected to:")
    for ep in connect_endpoints:
        print("  ", ep)

    print("SUB subscriptions:")
    for p in subscribe_prefixes:
        print("  ", p)

    while True:
        topic_b, fb_buf = sub.recv_multipart()
        t = Telemetry.Telemetry.GetRootAsTelemetry(fb_buf, 0)

        print(
            f"[{topic_b.decode()}] "
            f"seq={t.Seq()} ts={t.Ts():.3f} temp={t.Temp():.2f} msg='{t.Msg()}'"
        )


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
    ]

    # Subscriber only wants these prefixes:
    subscribe_prefixes = ["telemetry/", "cam/"]

    pub_procs: list[mp.Process] = []
    sub_proc: mp.Process | None = None

    try:
        for p in publishers:
            proc = mp.Process(
                target=publisher_proc,
                args=(p["name"], p["bind"], p["topics"]),
                daemon=True,
            )
            proc.start()
            pub_procs.append(proc)

        sub_proc = mp.Process(
            target=subscriber_proc,
            args=([p["bind"] for p in publishers], subscribe_prefixes),
            daemon=True,
        )
        sub_proc.start()

        # Keep parent alive
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
