import asyncio


async def slow_task():
    await asyncio.sleep(2)
    return "done"


async def main():
    try:
        result = await asyncio.wait_for(slow_task(), timeout=1.0)
        print(result)
    except asyncio.TimeoutError:
        print("Timed out!")


asyncio.run(main())
