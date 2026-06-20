import gi
import time

gi.require_version("Gst", "1.0")
gi.require_version("GstVideo", "1.0")

from gi.repository import Gst, GstVideo

Gst.init(None)

pipeline_str = (
    "v4l2src device=/dev/video0 ! "
    "capsfilter name=source_filter "
    "caps=video/x-raw,format=YUY2,width=640,height=480,framerate=30/1 ! "
    "videoconvert ! "
    "x264enc name=encoder "
    "bitrate=300 speed-preset=ultrafast tune=zerolatency "
    "key-int-max=30 bframes=0 byte-stream=true ! "
    "h264parse config-interval=1 ! "
    "rtph264pay pt=96 mtu=1400 config-interval=1 ! "
    "udpsink host=127.0.0.1 port=5600 sync=false async=false"
)

pipeline = Gst.parse_launch(pipeline_str)

source_filter = pipeline.get_by_name("source_filter")
encoder = pipeline.get_by_name("encoder")


def force_keyframe():
    sinkpad = encoder.get_static_pad("sink")

    event = GstVideo.video_event_new_downstream_force_key_unit(
        Gst.CLOCK_TIME_NONE,
        Gst.CLOCK_TIME_NONE,
        0,
        True,
        0,
    )

    sinkpad.send_event(event)


def wait_state(state):
    pipeline.set_state(state)
    ret, current, pending = pipeline.get_state(5 * Gst.SECOND)

    if ret == Gst.StateChangeReturn.FAILURE:
        raise RuntimeError(f"Failed to change pipeline state to {state.value_nick}")


def switch_camera_mode(width, height, bitrate_kbps):
    print(f"\n[SWITCH] camera={width}x{height}, bitrate={bitrate_kbps} kbps")

    # Pause the pipeline so v4l2src can renegotiate camera caps
    wait_state(Gst.State.PAUSED)

    new_caps = Gst.Caps.from_string(
        f"video/x-raw,format=YUY2,width={width},height={height},framerate=30/1"
    )

    source_filter.set_property("caps", new_caps)
    encoder.set_property("bitrate", bitrate_kbps)

    # Resume streaming
    wait_state(Gst.State.PLAYING)

    # Help receiver update after mode change
    force_keyframe()


try:
    wait_state(Gst.State.PLAYING)
    print("Sender started: 640x480 @ 300 kbps")

    while True:
        time.sleep(6)
        switch_camera_mode(640, 360, 200)

        time.sleep(6)
        switch_camera_mode(640, 480, 300)

except KeyboardInterrupt:
    print("\nStopping...")

finally:
    pipeline.set_state(Gst.State.NULL)