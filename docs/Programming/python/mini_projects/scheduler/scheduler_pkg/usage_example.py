from __future__ import annotations

import sys
import time
from dataclasses import dataclass, field
from typing import cast

from loguru import logger

logger.remove()
logger.add(sys.stderr, level="DEBUG")


try:
    from . import  Command, CommandScheduler, SchedulerContext
except ImportError:
    from commands import  Command, SchedulerContext
    from scheduler import CommandScheduler

@dataclass
class BasicSchedulerContext:
    last_error: BaseException | None = None

@dataclass
class AppContext(BasicSchedulerContext):
    messages: list[str] = field(default_factory=list)


@dataclass
class PrintCommand(Command):
    message: str

    def execute(self, context: SchedulerContext) -> None:
        app_context = cast(AppContext, context)
        app_context.messages.append(self.message)
        logger.info(self.message)


REPEAT_CMD_KEY = "printer"

def main() -> None:
    context = AppContext()
    scheduler = CommandScheduler(name="my-scheduler", context=context)
    scheduler.start()

    try:
        cmd_once = PrintCommand(message="runs once")
        cmd_repeated = PrintCommand(message="runs repeatedly")
        scheduler.submit(cmd_once, delay_s=1.0)
        scheduler.schedule(cmd_repeated, interval_s=0.5, key=REPEAT_CMD_KEY)
        time.sleep(2.0)
        scheduler.remove(REPEAT_CMD_KEY)
    finally:
        scheduler.stop()

    print(f"messages stored in context: {len(context.messages)}")


if __name__ == "__main__":
    main()
