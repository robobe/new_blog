#!/usr/bin/env python3
import msgpack
import numpy as np
import zmq


def validate(metadata: dict, payload_size: int) -> None:
    required = {
        "version", "frame_id", "timestamp_ns", "width", "height",
        "channels", "dtype", "encoding", "payload_size",
    }
    if not required.issubset(metadata):
        raise ValueError("metadata fields are missing")
    if metadata["version"] != 1:
        raise ValueError("unsupported version")
    if metadata["dtype"] != "uint8" or metadata["encoding"] != "raw":
        raise ValueError("only raw uint8 images are supported")
    if metadata["channels"] not in (1, 3):
        raise ValueError("only one or three channels are supported")

    expected = metadata["width"] * metadata["height"] * metadata["channels"]
    if metadata["width"] <= 0 or metadata["height"] <= 0:
        raise ValueError("image dimensions must be positive")
    if metadata["payload_size"] != expected or payload_size != expected:
        raise ValueError("payload size does not match image metadata")


def main() -> None:
    context = zmq.Context()
    server = context.socket(zmq.REP)
    server.bind("tcp://*:5555")

    metadata_frame, pixel_frame = server.recv_multipart()
    try:
        metadata = msgpack.unpackb(metadata_frame, raw=False)
        validate(metadata, len(pixel_frame))

        shape = (metadata["height"], metadata["width"])
        if metadata["channels"] == 3:
            shape += (3,)
        image = np.frombuffer(pixel_frame, dtype=np.uint8).reshape(shape)

        reply = f"OK frame_id={metadata['frame_id']} checksum={int(image.sum())}"
        server.send_string(reply)
        print(reply)
    except (KeyError, TypeError, ValueError, msgpack.exceptions.UnpackException) as error:
        reply = f"ERROR: {error}"
        server.send_string(reply)
        raise
    finally:
        server.close()
        context.term()


if __name__ == "__main__":
    main()
