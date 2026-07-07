from __future__ import annotations

import sys
import time
from dataclasses import dataclass, field
from typing import cast

from loguru import logger

try:
    from .commands import Command, SchedulerContext
    from .scheduler import CommandScheduler
except ImportError:
    from commands import Command, SchedulerContext
    from scheduler import CommandScheduler


logger.remove()
logger.add(sys.stderr, level="DEBUG")


@dataclass
class AppContext:
    last_error: BaseException | None = None
    errors: list[str] = field(default_factory=list)
    runs: int = 0


@dataclass
class FailsEverySecondCallCommand(Command):
    job_name: str

    def execute(self, context: SchedulerContext) -> None:
        app_context = cast(AppContext, context)
        app_context.runs += 1
        logger.info("{} run #{}", self.job_name, app_context.runs)

        if app_context.runs % 2 == 0:
            app_context.errors.append(f"{self.job_name} failed on run #{app_context.runs}")
            raise RuntimeError(f"{self.job_name} failed on run #{app_context.runs}")


def handle_scheduler_error(exc: BaseException, command: Command) -> None:
    logger.error("command {} failed: {}", command.__class__.__name__, exc)


def main() -> None:
    command_key = "nightly-report"
    context = AppContext()
    scheduler = CommandScheduler(context, on_error=handle_scheduler_error)
    scheduler.start()

    try:
        scheduler.schedule(
            FailsEverySecondCallCommand(job_name="nightly-report"),
            interval_s=1.0,
            key=command_key,
        )
        time.sleep(5.0)
        scheduler.remove(command_key)
        
    finally:
        scheduler.stop()
        logger.info("scheduler stopped")

    logger.info("total command runs: {}", context.runs)
    logger.info("errors recorded in context: {}", len(context.errors))


if __name__ == "__main__":
    main()
