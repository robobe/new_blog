import zmq
import time
import msgpack
import multiprocessing as mp

ENDPOINT = "ipc:///tmp/dealer.ipc"


def server():
    ctx = zmq.Context()
    sock = ctx.socket(zmq.ROUTER)
    sock.bind(ENDPOINT)

    print("Server: waiting for message...")

    # ROUTER receives multipart: [identity][data]
    identity, data = sock.recv_multipart()

    msg = msgpack.unpackb(data, raw=False)
    print("Server received:", msg)

    reply = {
        "status": "ok",
        "received_id": msg["id"],
    }

    # MUST send identity back
    sock.send_multipart([
        identity,
        msgpack.packb(reply),
    ])

    print("Server: reply sent")

    sock.close()
    ctx.term()

def client():
    ctx = zmq.Context()
    sock = ctx.socket(zmq.DEALER)

    # Optional but recommended: explicit identity
    sock.setsockopt(zmq.IDENTITY, b"client-1")

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
    mp.set_start_method("spawn", force=True)

    p_server = mp.Process(target=server)
    p_client = mp.Process(target=client)

    p_server.start()
    time.sleep(0.1)   # server first
    p_client.start()

    p_client.join()
    p_server.join()
