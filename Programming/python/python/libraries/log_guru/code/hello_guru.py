from loguru import logger
import sys

logger.remove()  # remove default handler
logger.add(sys.stdout, level="DEBUG")

logger.debug("Debug message")
logger.info("Info message")
logger.warning("Warning message")
logger.error("Error message")
logger.critical("Critical message")