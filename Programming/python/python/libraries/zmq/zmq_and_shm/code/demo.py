import zmq
import numpy as np
import time
import multiprocessing as mp
from multiprocessing import shared_memory
from dataclasses import dataclass, asdict
from typing import Tuple, Dict, Any, cast

# ================= CONFIG =================
HEIGHT = 480
WIDTH = 640
CHANNELS = 3
DTYPE = np.uint8
SHAPE = (HEIGHT, WIDTH, CHANNELS)

ENDPOINT = "ipc:///tmp/image_stream.ipc"
FPS = 5
NUM_FRAMES = 10
# =========================================


# ================= METADATA =================
@dataclass(frozen=True)
class ImageMeta:
    shm_name: str
    shape: Tuple[int, int, int]
    dtype: str
    frame_id: int
    timestamp: float

    @classmethod
    def create(cls, shm_name, shape, dtype, frame_id):
        return cls(
            shm_name=shm_name,
            shape=shape,
            dtype=dtype,
            frame_id=frame_id,
            timestamp=time.time(),
        )
# ===========================================


def publisher():
    """Generate images → write to SHM → notify via ZMQ"""

    size = np.prod(SHAPE) * np.dtype(DTYPE).itemsize

    # Create SHM once
    shm = shared_memory.SharedMemory(create=True, size=int(size))
    img = np.ndarray(SHAPE, dtype=DTYPE, buffer=shm.buf)

    ctx = zmq.Context()
    pub = ctx.socket(zmq.PUB)
    pub.bind(ENDPOINT)

    time.sleep(0.3)  # allow subscriber to connect

    for frame_id in range(NUM_FRAMES):
        # Generate new image directly INTO SHM
        img[:] = np.random.randint(0, 256, SHAPE, dtype=DTYPE)

        meta = ImageMeta.create(
            shm_name=shm.name,
            shape=SHAPE,
            dtype="uint8",
            frame_id=frame_id,
        )

        pub.send_json(asdict(meta))
        print(f"[PUB] frame {frame_id}")

        time.sleep(1.0 / FPS)

    pub.close()
    ctx.term()
    shm.close()
    shm.unlink()


def subscriber():
    """Wait for ZMQ signal → read image from SHM"""

    ctx = zmq.Context()
    sub = ctx.socket(zmq.SUB)
    sub.connect(ENDPOINT)
    sub.setsockopt(zmq.SUBSCRIBE, b"")

    last_frame = -1

    while True:
        raw: Dict = cast(Dict[str, Any], sub.recv_json())
        meta = ImageMeta(**raw)

        shm = shared_memory.SharedMemory(name=meta.shm_name)
        img = np.ndarray(
            meta.shape,
            dtype=np.dtype(meta.dtype),
            buffer=shm.buf,
        )

        # Detect dropped frames (normal behavior)
        if last_frame != -1 and meta.frame_id != last_frame + 1:
            print(f"[SUB] dropped {last_frame + 1}..{meta.frame_id - 1}")

        last_frame = meta.frame_id

        print(
            f"[SUB] frame {meta.frame_id}, "
            f"first pixel = {img[0, 0]}"
        )

        shm.close()

        if meta.frame_id >= NUM_FRAMES - 1:
            break

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
