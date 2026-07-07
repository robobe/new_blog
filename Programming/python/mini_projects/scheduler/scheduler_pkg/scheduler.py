from __future__ import annotations

import threading
import time

from loguru import logger

try:
    from .cancellation import CancellationRegistry
    from .commands import  Command, ScheduledCommand, SchedulerContext
    from .scheduler_queue import TimedCommandQueue
    from .worker import ErrorCallback, SchedulerWorker
except ImportError:
    from cancellation import CancellationRegistry
    from commands import  Command, ScheduledCommand, SchedulerContext
    from scheduler_queue import TimedCommandQueue
    from worker import ErrorCallback, SchedulerWorker


class CommandScheduler:
    def __init__(
        self,
        context: SchedulerContext | None = None,
        on_error: ErrorCallback | None = None,
        name: str = "scheduler",
    ) -> None:
        self.name = name
        self.context = context
        self._queue = TimedCommandQueue()
        self._cancellation = CancellationRegistry()
        self._lock = threading.Lock()
        self._wake_event = threading.Event()
        self._stop_event = threading.Event()
        self._worker = SchedulerWorker(
            queue=self._queue,
            cancellation=self._cancellation,
            lock=self._lock,
            wake_event=self._wake_event,
            stop_event=self._stop_event,
            context=self.context,
            set_last_error=self._set_last_error,
            on_error=on_error,
        )
        self._thread: threading.Thread | None = None

    # region Public API
    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            logger.info("scheduler already started name={}", self.name)
            return

        self._stop_event.clear()
        self._thread = threading.Thread(
            target=self._worker.run,
            daemon=True,
            name=f"{self.name}_t",
        )
        self._thread.start()
        logger.info("scheduler started name={}", self.name)

    def stop(self, timeout: float | None = 2.0) -> None:
        logger.info("scheduler stopping name={}", self.name)
        self._stop_event.set()
        self._wake_event.set()
        if self._thread is not None:
            self._thread.join(timeout=timeout)
            self._thread = None
        logger.info("scheduler stopped name={}", self.name)

    def submit(self, command: Command, delay_s: float = 0.0) -> None:
        """
        submit a command to be executed after a delay or immediately.
        """
        token = self._cancellation.new_token(command)
        self._submit(command, delay_s=delay_s, token=token, replace=True)

    def schedule(
        self,
        command: Command,
        interval_s: float,
        delay_s: float = 0.0,
        key: str | None = None,
    ) -> None:
        """
        schedule a command to be executed repeatedly at a given interval.
        """
        if interval_s <= 0:
            raise ValueError("interval_s must be > 0")

        self.submit(
            ScheduledCommand(
                command=command,
                repeat_interval_s=interval_s,
                key_override=key,
            ),
            delay_s=delay_s,
        )

    def remove(self, key: str) -> None:
        """
        remove a scheduled command by its key.
        """
        with self._lock:
            self._cancellation.remove(key)
        logger.debug("scheduler canceled command name={} key={}", self.name, key)
        self._wake_event.set()
    # endregion

    def _submit(
        self,
        command: Command,
        delay_s: float,
        token: object | None,
        replace: bool,
    ) -> None:
        """
        submit command to schduler queue
        """
        run_at = time.monotonic() + delay_s
        with self._lock:
            if replace:
                self._cancellation.activate(command, token)
            self._queue.push(run_at, token, command)
        logger.debug(
            "scheduler added command name={} command={} delay_s={} run_at={} repeat_interval_s={}",
            self.name,
            command.__class__.__name__,
            delay_s,
            run_at,
            command.repeat_interval_s,
        )
        self._wake_event.set()

    def _set_last_error(self, exc: BaseException) -> None:
        self.context.last_error = exc
