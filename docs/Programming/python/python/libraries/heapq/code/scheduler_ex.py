import heapq
import logging
import time
import threading
from dataclasses import dataclass, field
from typing import Callable


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


class Scheduler:
    def __init__(self):
        self.tasks: list[ScheduledTask] = []
        self.lock = threading.Lock()

    def schedule(self, delay_sec: float, name: str, callback: Callable[[], None]):
        run_at = time.monotonic() + delay_sec

        with self.lock:
            heapq.heappush(
                self.tasks,
                ScheduledTask(run_at, name, callback),
            )

    def run_pending(self):
        now = time.monotonic()
        ready_tasks: list[ScheduledTask] = []

        # Take ready tasks quickly, then release lock
        with self.lock:
            while self.tasks and self.tasks[0].run_at <= now:
                ready_tasks.append(heapq.heappop(self.tasks))

        # Run callbacks outside the lock
        for task in ready_tasks:
            task.callback()


def producer_thread(scheduler: Scheduler, stop_event: threading.Event):
    counter = 0

    while not stop_event.is_set():
        counter += 1

        scheduler.schedule(
            delay_sec=0.0,
            name=f"task_{counter}",
            callback=lambda i=counter: logging.info("hello from task %s", i),
        )

        time.sleep(0.5)


scheduler = Scheduler()
stop_event = threading.Event()

producer = threading.Thread(
    target=producer_thread,
    args=(scheduler, stop_event),
)

producer.start()

try:
    # Main loop, 50 Hz
    while True:
        scheduler.run_pending()
        time.sleep(0.02)

except KeyboardInterrupt:
    logging.info("stopping...")

finally:
    stop_event.set()
    producer.join()
