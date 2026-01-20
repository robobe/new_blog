import asyncio
import logging
logging.basicConfig(
    format='%(asctime)s - %(levelname)s - %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S',
    level=logging.DEBUG)

async def say_hello_async():
    logging.debug("Preparing Async World! and go to sleep...")
    await asyncio.sleep(2)  # Simulates waiting for 2 seconds
    logging.debug("Hello, Async World!")

async def do_something_else():
    logging.debug("Starting another task...")
    await asyncio.sleep(1)  # Simulates doing something else for 1 second
    logging.debug("Finished another task!")

async def main():
    # Schedule both tasks to run concurrently
    await asyncio.gather(
        say_hello_async(),
        do_something_else(),
    )

asyncio.run(main())