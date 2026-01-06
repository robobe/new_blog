import zmq
import time
import msgpack
import multiprocessing as mp

ENDPOINT = "ipc:///tmp/pair.ipc"

def publisher():
    ctx = zmq.Context()
    sock = ctx.socket(zmq.PAIR)
    sock.bind(ENDPOINT)

    print("Server: waiting for message...")
    data = sock.recv()

    msg = msgpack.unpackb(data, raw=False)
    print("Server received:", msg)

    reply = {
        "status": "ok",
        "received_id": msg["id"],
    }

    sock.send(msgpack.packb(reply))
    print("Server: reply sent")

    sock.close()
    ctx.term()

def subscriber():
    ctx = zmq.Context()
    sock = ctx.socket(zmq.PAIR)
    sock.connect(ENDPOINT)

    msg = {
        "id": 1,
        "name": "camera",
        "value": 3.14,
        "active": True,
    }

    sock.send(msgpack.packb(msg))
    print("Client: message sent")

    reply = sock.recv()
    reply_msg = msgpack.unpackb(reply, raw=False)
    print("Client received:", reply_msg)

    sock.close()
    ctx.term()

if __name__ == "__main__":
    mp.set_start_method("spawn", force=True)  # safe default

    p_sub = mp.Process(target=subscriber)
    p_pub = mp.Process(target=publisher)

    p_sub.start()
    time.sleep(0.1)   # ensure SUB starts first
    p_pub.start()

    p_pub.join()
    p_sub.join()
