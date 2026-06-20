import queue
import threading
from concurrent.futures import Future
from dataclasses import dataclass
from typing import Any

import gi

gi.require_version("Gst", "1.0")
gi.require_version("GstVideo", "1.0")
gi.require_version("GLib", "2.0")

from gi.repository import GLib, Gst, GstVideo


Gst.init(None)


CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
DEFAULT_WIDTH = 640
DEFAULT_HEIGHT = 480
DEFAULT_FPS = 30
DEFAULT_BITRATE_KBPS = 300
DEFAULT_KEY_INT_MAX = 30
SUPPORTED_KEY_INT_MAX = (30, 60, 90)
DEFAULT_VBV_BUF_CAPACITY = 100

CMD_INIT = "init"
CMD_SHUTDOWN = "shutdown"
CMD_STATUS = "status"
CMD_SET_STREAM = "set_stream"


@dataclass
class Command:
    name: str
    args: dict[str, Any]
    future: Future


class StreamController:
    def __init__(self):
        self.commands: queue.Queue[Command] = queue.Queue()
        self.context = GLib.MainContext()
        self.loop = GLib.MainLoop.new(self.context, False)
        self.thread = threading.Thread(target=self._thread_main, daemon=True)

        self.pipeline = None
        self.crop = None
        self.capsfilter = None
        self.encoder = None
        self.started = False

        self.width = DEFAULT_WIDTH
        self.height = DEFAULT_HEIGHT
        self.fps = DEFAULT_FPS
        self.bitrate_kbps = DEFAULT_BITRATE_KBPS
        self.key_int_max = DEFAULT_KEY_INT_MAX

    def start(self):
        self.thread.start()
        return self.call_sync(CMD_INIT, timeout=5)

    def stop(self):
        try:
            self.call_sync(CMD_SHUTDOWN, timeout=5)
        finally:
            self._invoke(lambda: self.loop.quit() or False)
            self.thread.join(timeout=2)

    def submit(self, name: str, args: dict | None = None) -> Future:
        future = Future()
        self.commands.put(Command(name=name, args=args or {}, future=future))
        self._invoke(self._process_commands)
        return future

    def call_sync(self, name: str, args: dict | None = None, timeout: float = 2):
        future = self.submit(name, args)
        return future.result(timeout=timeout)

    def _invoke(self, callback):
        self.context.invoke_full(GLib.PRIORITY_DEFAULT, callback)

    def _thread_main(self):
        self.context.push_thread_default()
        try:
            self.loop.run()
        finally:
            self.context.pop_thread_default()

    def _process_commands(self):
        while True:
            try:
                cmd = self.commands.get_nowait()
            except queue.Empty:
                break

            try:
                cmd.future.set_result(self._handle_command(cmd))
            except Exception as exc:
                cmd.future.set_exception(exc)

        return False

    def _handle_command(self, cmd: Command):
        if cmd.name == CMD_INIT:
            return self._init_pipeline()
        if cmd.name == CMD_SHUTDOWN:
            return self._shutdown_pipeline()
        if cmd.name == CMD_STATUS:
            return {"ok": True, **self._status_fields()}
        if cmd.name == CMD_SET_STREAM:
            return self._set_stream(**cmd.args)

        raise RuntimeError(f"Unknown command: {cmd.name}")

    def _init_pipeline(self):
        if self.started:
            return {"ok": True, "already_started": True, **self._status_fields()}

        self.pipeline = Gst.parse_launch(
            "v4l2src device=/dev/video0 ! "
            "video/x-raw,format=YUY2,width=640,height=480,framerate=30/1 ! "
            "videocrop name=center_crop ! "
            "videorate name=rate drop-only=true ! "
            "capsfilter name=stream_caps ! "
            "videoconvert ! "
            "x264enc name=encoder "
            "bitrate=300 speed-preset=ultrafast tune=zerolatency "
            "key-int-max=30 vbv-buf-capacity=1000 "
            "bframes=0 byte-stream=true ! "
            "h264parse config-interval=1 ! "
            "rtph264pay pt=96 mtu=1400 config-interval=1 ! "
            "udpsink host=127.0.0.1 port=5600 sync=false async=false"
        )

        self.crop = self.pipeline.get_by_name("center_crop")
        self.capsfilter = self.pipeline.get_by_name("stream_caps")
        self.encoder = self.pipeline.get_by_name("encoder")

        if self.crop is None or self.capsfilter is None or self.encoder is None:
            raise RuntimeError("Could not find required GStreamer elements")

        self._apply_stream_settings(
            self.width,
            self.height,
            self.fps,
            self.bitrate_kbps,
            self.key_int_max,
            force_keyframe=False,
        )

        bus = self.pipeline.get_bus()
        bus.add_signal_watch()
        bus.connect("message", self._on_bus_message)

        ret = self.pipeline.set_state(Gst.State.PLAYING)
        if ret == Gst.StateChangeReturn.FAILURE:
            raise RuntimeError("Failed to start bandwidth control pipeline")

        self.started = True
        return {"ok": True, **self._status_fields()}

    def _shutdown_pipeline(self):
        if self.pipeline is not None:
            self.pipeline.set_state(Gst.State.NULL)
            self.pipeline = None
            self.crop = None
            self.capsfilter = None
            self.encoder = None

        self.started = False
        return {"ok": True, **self._status_fields()}

    def _set_stream(
        self,
        width: int,
        height: int,
        fps: int,
        bitrate_kbps: int,
        key_int_max: int,
    ):
        if not self.started:
            raise RuntimeError("Pipeline is not running")

        self._validate_stream_settings(width, height, fps, bitrate_kbps, key_int_max)
        self._apply_stream_settings(width, height, fps, bitrate_kbps, key_int_max)

        self.width = width
        self.height = height
        self.fps = fps
        self.bitrate_kbps = bitrate_kbps
        self.key_int_max = key_int_max

        return {"ok": True, **self._status_fields()}

    def _validate_stream_settings(
        self,
        width: int,
        height: int,
        fps: int,
        bitrate_kbps: int,
        key_int_max: int,
    ):
        if width <= 0 or width > CAMERA_WIDTH:
            raise ValueError(f"width must be between 1 and {CAMERA_WIDTH}")
        if height <= 0 or height > CAMERA_HEIGHT:
            raise ValueError(f"height must be between 1 and {CAMERA_HEIGHT}")
        if fps <= 0 or fps > DEFAULT_FPS:
            raise ValueError(f"fps must be between 1 and {DEFAULT_FPS}")
        if bitrate_kbps <= 0:
            raise ValueError("bitrate_kbps must be greater than 0")
        if key_int_max not in SUPPORTED_KEY_INT_MAX:
            supported = ", ".join(map(str, SUPPORTED_KEY_INT_MAX))
            raise ValueError(f"key_int_max must be one of: {supported}")

    def _apply_stream_settings(
        self,
        width: int,
        height: int,
        fps: int,
        bitrate_kbps: int,
        key_int_max: int,
        force_keyframe: bool = True,
    ):
        crop = self._center_crop_values(width, height)

        self.crop.set_property("left", crop["left"])
        self.crop.set_property("right", crop["right"])
        self.crop.set_property("top", crop["top"])
        self.crop.set_property("bottom", crop["bottom"])

        caps = Gst.Caps.from_string(
            f"video/x-raw,width={width},height={height},framerate={fps}/1"
        )
        self.capsfilter.set_property("caps", caps)
        self.encoder.set_property("bitrate", bitrate_kbps)
        self.encoder.set_property("key-int-max", key_int_max)

        if force_keyframe:
            self._force_keyframe()

    def _force_keyframe(self):
        sinkpad = self.encoder.get_static_pad("sink")
        event = GstVideo.video_event_new_downstream_force_key_unit(
            Gst.CLOCK_TIME_NONE,
            Gst.CLOCK_TIME_NONE,
            0,
            True,
            0,
        )
        sinkpad.send_event(event)

    def _status_fields(self):
        crop = self._center_crop_values(self.width, self.height)
        return {
            "started": self.started,
            "camera_width": CAMERA_WIDTH,
            "camera_height": CAMERA_HEIGHT,
            "width": self.width,
            "height": self.height,
            "fps": self.fps,
            "bitrate_kbps": self.bitrate_kbps,
            "key_int_max": self.key_int_max,
            "vbv_buf_capacity": DEFAULT_VBV_BUF_CAPACITY,
            "crop": crop,
            "udp_host": "127.0.0.1",
            "udp_port": 5600,
            "has_pipeline": self.pipeline is not None,
            "has_crop": self.crop is not None,
            "has_capsfilter": self.capsfilter is not None,
            "has_encoder": self.encoder is not None,
        }

    def _center_crop_values(self, width: int, height: int):
        left = (CAMERA_WIDTH - width) // 2
        right = CAMERA_WIDTH - width - left
        top = (CAMERA_HEIGHT - height) // 2
        bottom = CAMERA_HEIGHT - height - top
        return {"left": left, "right": right, "top": top, "bottom": bottom}

    def _on_bus_message(self, bus, message):
        if message.type == Gst.MessageType.ERROR:
            err, debug = message.parse_error()
            print(f"GStreamer error: {err}")
            if debug:
                print(f"GStreamer debug: {debug}")
        elif message.type == Gst.MessageType.EOS:
            print("GStreamer EOS")
