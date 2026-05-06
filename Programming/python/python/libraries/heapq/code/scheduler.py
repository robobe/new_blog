import heapq
import logging
import time
from dataclasses import dataclass, field
from typing import Callable, Optional


logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(levelname)s %(message)s",
    datefmt="%H:%M:%S",
)


@dataclass(order=True)
class ScheduledTask:
    run_at: float
    name: str = field(compare=False)
    callback: Callable[[], None] = field(compare=False)
    interval: Optional[float] = field(default=None, compare=False)  # seconds


class Scheduler:
    def __init__(self):
        self.tasks: list[ScheduledTask] = []

    def schedule(self, delay_sec: float, name: str, callback: Callable[[], None], interval: Optional[float] = None):
        run_at = time.monotonic() + delay_sec
        heapq.heappush(self.tasks, ScheduledTask(run_at, name, callback, interval))
        # logging.info("scheduled task=%s delay=%.2fs interval=%s", name, delay_sec, interval)

    def run_pending(self):
        now = time.monotonic()

        while self.tasks and self.tasks[0].run_at <= now:
            task = heapq.heappop(self.tasks)

            # logging.info("running task=%s", task.name)
            task.callback()

            # Re-schedule if periodic.
            if task.interval is not None:
                task.run_at = now + task.interval
                heapq.heappush(self.tasks, task)
                # logging.info("rescheduled task=%s interval=%.2fs", task.name, task.interval)

def tick():
    logging.info("tick")

def slow():
    logging.info("slow task")

scheduler = Scheduler()

scheduler.schedule(0.0, "tick_1hz", tick, interval=1.0)   # every 1 sec
scheduler.schedule(0.0, "slow_3s", slow, interval=3.0)    # every 3 sec
scheduler.schedule(2.0, "one_shot", lambda: logging.info("once"))


while True:
    scheduler.run_pending()
    time.sleep(0.02)  # 50 Hz loop
