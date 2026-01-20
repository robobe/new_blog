import asyncio


async def worker(name, delay):
    await asyncio.sleep(delay)
    return f"{name} finished"


async def main():
    tasks = [
        asyncio.create_task(worker("A", 2)),
        asyncio.create_task(worker("B", 1)),
        asyncio.create_task(worker("C", 3)),
    ]

    done, pending = await asyncio.wait(
        tasks,
        return_when=asyncio.FIRST_COMPLETED,
    )

    # One or more tasks finished
    for task in done:
        print("First result:", task.result())

    # Optional: cancel the rest
    for task in pending:
        task.cancel()

    await asyncio.gather(*pending, return_exceptions=True)


asyncio.run(main())
