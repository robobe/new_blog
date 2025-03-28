import multiprocessing
import logging
from dataclasses import dataclass, asdict
import msgpack
import zmq

FMT = "%(asctime)s - %(lineno)s - %(levelname)s - %(message)s"
logging.basicConfig(format=FMT, level=logging.INFO)

log = logging.getLogger(__name__)

TOPIC = b"topic"
IPC_PATH = "/tmp/zmq-ipc"


@dataclass
class Data_Request:
    f_int: int
    f_float: float
    f_string: str


@dataclass
class Data_Response:
    success: bool


def server():
    context = zmq.Context()
    socket = context.socket(zmq.REP)
    socket.bind(f"ipc://{IPC_PATH}")
    topic, data = socket.recv_multipart()

    msg = msgpack.unpackb(data)
    log.info(f"server get request: {msg}")

    response = Data_Response(success=True)
    data = msgpack.packb(asdict(response))
    socket.send(data)


def client():
    context = zmq.Context()
    socket = context.socket(zmq.REQ)
    socket.connect(f"ipc://{IPC_PATH}")

    # Create request msg
    msg = Data_Request(1, 2.0, "string")
    raw = asdict(msg)
    data = msgpack.packb(raw)
    socket.send_multipart([TOPIC, data])

    # Recv response from server
    data = socket.recv()
    # unpack socket data
    raw = msgpack.unpackb(data)
    # Convert to msg
    msg = Data_Response(**raw)
    log.info(f"server response: {msg.success}")


if __name__ == "__main__":
    p_server = multiprocessing.Process(target=server)
    p_client = multiprocessing.Process(target=client)
    p_client.start()
    p_server.start()

    p_server.join()
    p_client.join()