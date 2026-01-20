import asyncio
import logging
logging.basicConfig(
    format='%(asctime)s - %(levelname)s - %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S',
    level=logging.DEBUG)


async def main():
    logging.debug("Hello ...")
    asyncio.get_event_loop().call_later(
        1.0,
        lambda: logging.debug("Immediate callback executed"))    
    await asyncio.sleep(2)
    logging.debug("... World!")

if __name__ == "__main__":
    asyncio.run(main())