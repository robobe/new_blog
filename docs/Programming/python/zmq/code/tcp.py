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