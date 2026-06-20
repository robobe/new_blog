import asyncio
from contextlib import asynccontextmanager
from pathlib import Path

import uvicorn
from fastapi import FastAPI, HTTPException
from fastapi.responses import FileResponse
from pydantic import BaseModel, Field

try:
    from .bandwidth_control import (
        CAMERA_HEIGHT,
        CAMERA_WIDTH,
        CMD_SET_STREAM,
        CMD_STATUS,
        DEFAULT_FPS,
        StreamController,
    )
except ImportError:
    from bandwidth_control import (
        CAMERA_HEIGHT,
        CAMERA_WIDTH,
        CMD_SET_STREAM,
        CMD_STATUS,
        DEFAULT_FPS,
        StreamController,
    )


STATIC_DIR = Path(__file__).with_name("static")

stream_controller = StreamController()


@asynccontextmanager
async def lifespan(app: FastAPI):
    stream_controller.start()
    try:
        yield
    finally:
        stream_controller.stop()


app = FastAPI(lifespan=lifespan)


class StreamRequest(BaseModel):
    width: int = Field(..., gt=0, le=CAMERA_WIDTH)
    height: int = Field(..., gt=0, le=CAMERA_HEIGHT)
    fps: int = Field(..., gt=0, le=DEFAULT_FPS)
    bitrate_kbps: int = Field(..., gt=0)
    key_int_max: int = Field(..., gt=0)


class ProfileRequest(BaseModel):
    profile: str


PROFILES = {
    "normal": {
        "width": 640,
        "height": 480,
        "fps": 30,
        "bitrate_kbps": 300,
        "key_int_max": 30,
    },
    "center_320x240": {
        "width": 320,
        "height": 240,
        "fps": 15,
        "bitrate_kbps": 100,
        "key_int_max": 30,
    },
}


async def await_stream_command(name: str, args: dict | None = None):
    try:
        future = stream_controller.submit(name, args)
        return await asyncio.wait_for(asyncio.wrap_future(future), timeout=3)
    except asyncio.TimeoutError:
        raise HTTPException(
            status_code=504,
            detail=f"GStreamer stream command timed out: {name}",
        )
    except ValueError as exc:
        raise HTTPException(status_code=400, detail=str(exc))
    except Exception as exc:
        raise HTTPException(status_code=500, detail=str(exc))


@app.get("/")
async def index():
    return FileResponse(STATIC_DIR / "bandwidth.html")


@app.get("/status")
async def status():
    return await await_stream_command(CMD_STATUS)


@app.get("/profiles")
async def profiles():
    return {"profiles": PROFILES}


@app.post("/stream")
async def set_stream(req: StreamRequest):
    return await await_stream_command(CMD_SET_STREAM, req.dict())


@app.post("/profile")
async def set_profile(req: ProfileRequest):
    if req.profile not in PROFILES:
        supported = ", ".join(PROFILES)
        raise HTTPException(
            status_code=400,
            detail=f"profile must be one of: {supported}",
        )

    return await await_stream_command(CMD_SET_STREAM, PROFILES[req.profile])


def main() -> None:
    uvicorn.run(app, host="0.0.0.0", port=8002)


if __name__ == "__main__":
    main()
