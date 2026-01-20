import asyncio


async def producer(fut: asyncio.Future):
    print("Producer: working...")
    await asyncio.sleep(1)

    # Fulfill the future
    fut.set_exception(RuntimeError("Something went wrong"))


async def consumer(fut: asyncio.Future):
    try:
        await fut
    except Exception as e:
        print("Consumer caught:", e)


async def main():
    loop = asyncio.get_running_loop()

    # Create an empty Future
    fut = loop.create_future()

    await asyncio.gather(
        producer(fut),
        consumer(fut),
    )


asyncio.run(main())
