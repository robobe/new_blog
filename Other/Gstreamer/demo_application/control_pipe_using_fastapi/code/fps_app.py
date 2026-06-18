import asyncio
from contextlib import asynccontextmanager
from pathlib import Path

import uvicorn
from fastapi import FastAPI, HTTPException
from fastapi.responses import FileResponse
from pydantic import BaseModel

from .fps_control import (
    CMD_SET_FPS,
    CMD_STATUS,
    FpsController,
)


STATIC_DIR = Path(__file__).with_name("static")

fps_controller = FpsController()


@asynccontextmanager
async def lifespan(app: FastAPI):
    fps_controller.start()
    try:
        yield
    finally:
        fps_controller.stop()


app = FastAPI(lifespan=lifespan)


class FpsRequest(BaseModel):
    fps: int


async def await_fps_command(name: str, args: dict | None = None):
    try:
        future = fps_controller.submit(name, args)
        return await asyncio.wait_for(
            asyncio.wrap_future(future),
            timeout=3,
        )
    except asyncio.TimeoutError:
        raise HTTPException(
            status_code=504,
            detail=f"GStreamer FPS command timed out: {name}",
        )
    except ValueError as exc:
        raise HTTPException(
            status_code=400,
            detail=str(exc),
        )
    except Exception as exc:
        raise HTTPException(
            status_code=500,
            detail=str(exc),
        )


@app.get("/")
async def index():
    return FileResponse(STATIC_DIR / "fps.html")


@app.get("/status")
async def status():
    return await await_fps_command(CMD_STATUS)


@app.post("/fps")
async def set_fps(req: FpsRequest):
    return await await_fps_command(
        CMD_SET_FPS,
        {"fps": req.fps},
    )


def main() -> None:
    uvicorn.run(app, host="0.0.0.0", port=8002)


if __name__ == "__main__":
    main()
