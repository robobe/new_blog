#!/usr/bin/env python3
"""Display raw and OpenCV-rectified Gazebo camera images."""

import argparse
import queue
import threading

import cv2
import numpy as np
from gz.msgs10.camera_info_pb2 import CameraInfo
from gz.msgs10.image_pb2 import BGR_INT8, L_INT8, RGB_INT8, Image
from gz.transport13 import Node


def image_to_cv(message: Image) -> np.ndarray:
    """Convert a supported gz.msgs.Image into an OpenCV image."""
    formats = {
        L_INT8: (1, None),
        RGB_INT8: (3, cv2.COLOR_RGB2BGR),
        BGR_INT8: (3, None),
    }
    if message.pixel_format_type not in formats:
        raise ValueError(
            f"Unsupported pixel format: {message.pixel_format_type}"
        )

    channels, conversion = formats[message.pixel_format_type]
    row_bytes = message.width * channels
    if message.step < row_bytes:
        raise ValueError(
            f"Image step {message.step} is smaller than {row_bytes}"
        )

    data = np.frombuffer(message.data, dtype=np.uint8)
    expected = message.step * message.height
    if data.size < expected:
        raise ValueError(f"Image has {data.size} bytes; expected {expected}")

    rows = data[:expected].reshape(message.height, message.step)
    pixels = rows[:, :row_bytes]
    if channels == 1:
        image = pixels.reshape(message.height, message.width)
    else:
        image = pixels.reshape(message.height, message.width, channels)

    return cv2.cvtColor(image, conversion) if conversion is not None else image


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "topic",
        nargs="?",
        default="camera_distortion",
        help="Gazebo image topic (default: camera_distortion)",
    )
    parser.add_argument(
        "--info-topic",
        help="Override the default <image topic>/camera_info convention",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    image_topic = args.topic.rstrip("/")
    info_topic = args.info_topic or f"{image_topic}/camera_info"

    frames: queue.Queue[tuple[np.ndarray, int, int]] = queue.Queue(maxsize=1)
    info_lock = threading.Lock()
    latest_info: CameraInfo | None = None
    node = Node()

    def on_image(message: Image) -> None:
        try:
            item = (image_to_cv(message), message.width, message.height)
            if frames.full():
                frames.get_nowait()
            frames.put_nowait(item)
        except (ValueError, queue.Empty, queue.Full) as error:
            print(f"Image callback error: {error}")

    def on_info(message: CameraInfo) -> None:
        nonlocal latest_info
        with info_lock:
            latest_info = message

    if not node.subscribe(Image, image_topic, on_image):
        raise RuntimeError(f"Could not subscribe to {image_topic!r}")
    if not node.subscribe(CameraInfo, info_topic, on_info):
        raise RuntimeError(f"Could not subscribe to {info_topic!r}")

    print(f"Image topic: {image_topic}")
    print(f"Camera info topic: {info_topic}")
    print("Press q or Esc to quit.")

    map_key: tuple[float, ...] | None = None
    map_x: np.ndarray | None = None
    map_y: np.ndarray | None = None

    try:
        while True:
            try:
                raw, width, height = frames.get(timeout=0.1)
            except queue.Empty:
                if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                    break
                continue

            with info_lock:
                info = latest_info

            if info is None:
                cv2.imshow("Waiting for camera_info", raw)
            else:
                if (info.width, info.height) != (width, height):
                    raise RuntimeError(
                        "Image dimensions do not match camera_info: "
                        f"image={width}x{height}, "
                        f"info={info.width}x{info.height}"
                    )
                if len(info.intrinsics.k) != 9:
                    raise RuntimeError("camera_info must contain a 3x3 K matrix")
                if len(info.distortion.k) < 4:
                    raise RuntimeError("camera_info has too few distortion values")

                camera_matrix = np.asarray(info.intrinsics.k, dtype=np.float64).reshape(3, 3)
                distortion = np.asarray(info.distortion.k, dtype=np.float64)
                key = (*camera_matrix.flat, *distortion, width, height)
                if key != map_key:
                    map_x, map_y = cv2.initUndistortRectifyMap(
                        camera_matrix,
                        distortion,
                        None,
                        camera_matrix,
                        (width, height),
                        cv2.CV_32FC1,
                    )
                    map_key = key

                rectified = cv2.remap(raw, map_x, map_y, cv2.INTER_LINEAR)
                raw_label = raw if raw.ndim == 3 else cv2.cvtColor(raw, cv2.COLOR_GRAY2BGR)
                fixed_label = (
                    rectified
                    if rectified.ndim == 3
                    else cv2.cvtColor(rectified, cv2.COLOR_GRAY2BGR)
                )
                cv2.putText(raw_label, "Raw", (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)
                cv2.putText(fixed_label, "Rectified", (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)
                cv2.imshow("Gazebo camera", np.hstack((raw_label, fixed_label)))

            if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                break
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
