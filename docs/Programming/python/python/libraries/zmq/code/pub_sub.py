import multiprocessing
import logging
from dataclasses import dataclass, asdict
import msgpack
import zmq
import time

FMT = "%(asctime)s - %(lineno)s - %(levelname)s - %(message)s"
logging.basicConfig(format=FMT, level=logging.INFO)
log = logging.getLogger(__name__)

TOPIC = b"topic"
SERVICE_PORT = 5555


@dataclass
class Data_Request:
    f_int: int
    f_float: float
    f_string: str


def publisher():
    context = zmq.Context()
    socket = context.socket(zmq.PUB)
    socket.bind(f"tcp://*:{SERVICE_PORT}")
    counter = 0
    while True:

        msg = Data_Request(counter, 2.0, "string")
        raw = asdict(msg)
        data = msgpack.packb(raw)

        socket.send_multipart([TOPIC, data])

        counter += 1
        time.sleep(1)  # ensure send completes


def subscriber():
    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    socket.connect(f"tcp://127.0.0.1:{SERVICE_PORT}")

    # Subscribe only to TOPIC
    socket.setsockopt(zmq.SUBSCRIBE, TOPIC)
    while True:
        topic, data = socket.recv_multipart()
        msg = msgpack.unpackb(data)

        log.info(f"subscriber received: {msg}")

if __name__ == "__main__":
    p_sub = multiprocessing.Process(target=subscriber)
    p_pub = multiprocessing.Process(target=publisher)

    p_sub.start()
    p_pub.start()

    p_pub.join()
    p_sub.join()
