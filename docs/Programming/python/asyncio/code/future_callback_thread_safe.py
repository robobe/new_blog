
import asyncio


def external_thread(loop, fut):
    loop.call_soon_threadsafe(fut.set_result, 99)


async def main():
    loop = asyncio.get_running_loop()
    fut = loop.create_future()

    fut.add_done_callback(lambda f: print("Result:", f.result()))

    import threading
    threading.Thread(target=external_thread, args=(loop, fut)).start()

    await fut


asyncio.run(main())
