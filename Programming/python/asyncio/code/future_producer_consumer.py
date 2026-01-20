import asyncio


async def producer(fut: asyncio.Future):
    print("Producer: working...")
    await asyncio.sleep(2)

    # Fulfill the future
    fut.set_result("Hello from the future!")


async def consumer(fut: asyncio.Future):
    print("Consumer: waiting for result...")
    result = await fut
    print(f"Consumer: got -> {result}")


async def main():
    loop = asyncio.get_running_loop()

    # Create an empty Future
    fut = loop.create_future()

    await asyncio.gather(
        producer(fut),
        consumer(fut),
    )


asyncio.run(main())
