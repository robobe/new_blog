from loguru import logger
import sys

#
logger.add("log.log", level="DEBUG")

logger.debug("Debug message")
logger.info("Info message")
logger.warning("Warning message")
logger.error("Error message")
logger.critical("Critical message")