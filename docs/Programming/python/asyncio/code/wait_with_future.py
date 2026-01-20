
import asyncio


async def main():
    loop = asyncio.get_running_loop()
    fut = loop.create_future()

    # Simulate external completion
    loop.call_later(2, fut.set_result, "ok")

    try:
        result = await asyncio.wait_for(fut, timeout=1)
        print(result)
    except asyncio.TimeoutError:
        print("Future timed out!")

asyncio.run(main())