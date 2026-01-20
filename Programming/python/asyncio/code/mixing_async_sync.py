import asyncio
import threading
import time


async def job(msg: str):
    await asyncio.sleep(0.2)
    print("Async job:", msg)


def worker(loop: asyncio.AbstractEventLoop):
    for i in range(5):
        time.sleep(0.5)
        loop.call_soon_threadsafe(asyncio.create_task, job(f"hello {i}"))


async def main():
    loop = asyncio.get_running_loop()
    threading.Thread(target=worker, args=(loop,), daemon=True).start()

    await asyncio.sleep(3)


asyncio.run(main())
