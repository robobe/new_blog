# GStreamer demo application

<div class="grid-container">
    <div class="grid-item">
        <a href="control_pipe_using_fastapi">
            <p>Control pipe using FastAPI</p>
        </a>
        <details>
            <summary>More ..</summary>
            <p>
                FastAPI demo that controls a running GStreamer pipeline from
                HTTP routes while the pipeline itself stays on a GLib thread.
                The example changes FPS through videorate and a capsfilter.
            </p>
        </details>
    </div>
    <div class="grid-item">
        <a href="control_video_bandwidth">
            <p>Control Video Bandwidth</p>
        </a>
        <details>
            <summary>More ..</summary>
            <p>
                Demo application for controlling video bandwidth with crop
                presets, FPS, encoder bitrate, keyframe interval, and measured
                RTP bandwidth from the running pipeline.
            </p>
        </details>
    </div>
    <div class="grid-item">
        <a href="switch_camera_source">
            <p>Switch Camera Source Mode</p>
        </a>
        <details>
            <summary>More ..</summary>
            <p>
                Demo application that changes the v4l2 camera caps while the
                pipeline is running, updates the encoder bitrate, sends RTP/H.264
                over UDP, and shows how to measure bandwidth with iftop.
            </p>
        </details>
    </div>
</div>
