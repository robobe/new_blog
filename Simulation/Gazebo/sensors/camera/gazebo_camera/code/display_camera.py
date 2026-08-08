#!/usr/bin/env python3
"""Display a Gazebo camera topic and its metadata with OpenCV."""

import argparse
from collections import deque
import queue
import threading
import time

import cv2
import numpy as np
from gz.msgs10.camera_info_pb2 import CameraInfo
from gz.msgs10.image_pb2 import PixelFormatType
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


def pixel_format_name(pixel_format: int) -> str:
    """Return a readable protobuf enum name."""
    try:
        return PixelFormatType.Name(pixel_format)
    except ValueError:
        return f"UNKNOWN({pixel_format})"


def distortion_model_name(model: int) -> str:
    """Return a readable camera distortion model name."""
    try:
        return CameraInfo.Distortion.DistortionModelType.Name(model)
    except ValueError:
        return f"UNKNOWN({model})"


def compact_values(values: list[float], limit: int = 5) -> str:
    """Format a protobuf numeric sequence for the metadata panel."""
    shown = ", ".join(f"{value:.3g}" for value in values[:limit])
    return f"[{shown}{', ...' if len(values) > limit else ''}]"


def metadata_view(
    frame: np.ndarray,
    *,
    image_topic: str,
    width: int,
    height: int,
    step: int,
    pixel_format: int,
    fps: float,
    info_topic: str,
    info: CameraInfo | None,
) -> np.ndarray:
    """Append a readable image and camera-info panel to a frame."""
    color_frame = (
        frame
        if frame.ndim == 3
        else cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
    )
    panel_width = 440
    canvas_height = max(color_frame.shape[0], 250)
    canvas = np.zeros(
        (canvas_height, color_frame.shape[1] + panel_width, 3), dtype=np.uint8
    )
    canvas[: color_frame.shape[0], : color_frame.shape[1]] = color_frame

    lines = [
        f"Image: {image_topic}",
        f"Size: {width} x {height}",
        f"Format: {pixel_format_name(pixel_format)}",
        f"Step: {step} bytes    FPS: {fps:.1f}",
        f"Info: {info_topic}",
    ]
    if info is None:
        lines.append("Waiting for camera_info...")
    else:
        lines.extend(
            [
                f"Info size: {info.width} x {info.height}",
                f"Distortion: {distortion_model_name(info.distortion.model)}",
                f"D: {compact_values(list(info.distortion.k))}",
                f"K: {compact_values(list(info.intrinsics.k), limit=9)}",
                f"P: {compact_values(list(info.projection.p), limit=4)}",
            ]
        )

    x = color_frame.shape[1] + 12
    for index, line in enumerate(lines):
        cv2.putText(
            canvas,
            line,
            (x, 24 + index * 22),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (230, 230, 230),
            1,
            cv2.LINE_AA,
        )
    return canvas


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "topic",
        nargs="?",
        default="camera",
        help="Gazebo image topic (default: camera)",
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

    frames: queue.Queue[tuple[np.ndarray, int, int, int, int]] = queue.Queue(
        maxsize=1
    )
    state_lock = threading.Lock()
    frame_times: deque[float] = deque(maxlen=120)
    latest_info: CameraInfo | None = None
    node = Node()

    def on_image(message: Image) -> None:
        try:
            item = (
                image_to_cv(message),
                message.width,
                message.height,
                message.step,
                message.pixel_format_type,
            )
            now = time.monotonic()
            with state_lock:
                frame_times.append(now)
                while frame_times and now - frame_times[0] > 1.0:
                    frame_times.popleft()
            if frames.full():
                frames.get_nowait()
            frames.put_nowait(item)
        except (ValueError, queue.Empty, queue.Full) as error:
            print(f"Image callback error: {error}")

    def on_info(message: CameraInfo) -> None:
        nonlocal latest_info
        info = CameraInfo()
        info.CopyFrom(message)
        with state_lock:
            latest_info = info

    if not node.subscribe(Image, image_topic, on_image):
        raise RuntimeError(f"Could not subscribe to {image_topic!r}")
    if not node.subscribe(CameraInfo, info_topic, on_info):
        raise RuntimeError(f"Could not subscribe to {info_topic!r}")

    print(f"Image topic: {image_topic}")
    print(f"Camera info topic: {info_topic}")
    print("Press q or Esc to quit.")
    try:
        while True:
            try:
                frame, width, height, step, pixel_format = frames.get(
                    timeout=0.1
                )
                with state_lock:
                    info = latest_info
                    fps = (
                        (len(frame_times) - 1)
                        / (frame_times[-1] - frame_times[0])
                        if len(frame_times) > 1
                        and frame_times[-1] > frame_times[0]
                        else 0.0
                    )
                view = metadata_view(
                    frame,
                    image_topic=image_topic,
                    width=width,
                    height=height,
                    step=step,
                    pixel_format=pixel_format,
                    fps=fps,
                    info_topic=info_topic,
                    info=info,
                )
                cv2.imshow("Gazebo camera", view)
            except queue.Empty:
                pass

            if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                break
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
