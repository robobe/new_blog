import asyncio


def on_ready(fut: asyncio.Future):
    print("Future is ready!")
    print("Result:", fut.result())


async def producer(fut: asyncio.Future):
    await asyncio.sleep(1)
    fut.set_result("done")


async def main():
    loop = asyncio.get_running_loop()
    fut = loop.create_future()

    fut.add_done_callback(on_ready)

    await producer(fut)


asyncio.run(main())
