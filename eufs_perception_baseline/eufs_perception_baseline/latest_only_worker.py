"""Latest-only worker with executor-serialized completion commits."""

import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Any, Callable, Deque, Optional


@dataclass(frozen=True)
class WorkerCompletion:
    """Result delivered to the ROS executor after background computation."""

    job: Any
    result: Any = None
    error: Optional[BaseException] = None


class LatestOnlyWorker:
    """Run one job at a time while retaining at most the newest pending job."""

    def __init__(
        self,
        node,
        compute: Callable[[Any], Any],
        commit: Callable[[WorkerCompletion], None],
        name: str,
    ) -> None:
        self._compute = compute
        self._commit = commit
        self._condition = threading.Condition()
        self._pending_job = None
        self._running = False
        self._stopping = False
        self._completions: Deque[WorkerCompletion] = deque()
        self._completion_guard = node.create_guard_condition(
            self._drain_completions
        )
        self._thread = threading.Thread(
            target=self._run,
            name=name,
            daemon=True,
        )
        self._thread.start()

    def submit(self, job: Any) -> bool:
        """Queue ``job``, replacing an older job that has not started yet."""
        with self._condition:
            if self._stopping:
                return False
            self._pending_job = job
            self._condition.notify()
            return True

    def clear_pending(self) -> None:
        """Discard work that has not started; a running job is fenced at commit."""
        with self._condition:
            self._pending_job = None

    def is_idle(self) -> bool:
        """Return whether compute, pending, and completion queues are empty."""
        with self._condition:
            return all(
                (
                    not self._running,
                    self._pending_job is None,
                    not self._completions,
                )
            )

    def wait_idle(self, timeout_sec: float = 5.0) -> bool:
        """Wait until compute and commit queues are empty for tests/support."""
        deadline = time.monotonic() + max(0.0, float(timeout_sec))
        while time.monotonic() < deadline:
            if self.is_idle():
                return True
            time.sleep(0.001)
        return self.is_idle()

    def shutdown(self) -> None:
        """Stop accepting jobs and join the worker after its current job ends."""
        with self._condition:
            if self._stopping:
                return
            self._stopping = True
            self._pending_job = None
            self._condition.notify_all()
        self._thread.join()

    def _run(self) -> None:
        while True:
            with self._condition:
                while self._pending_job is None and not self._stopping:
                    self._condition.wait()
                if self._stopping:
                    return
                job = self._pending_job
                self._pending_job = None
                self._running = True

            try:
                completion = WorkerCompletion(job=job, result=self._compute(job))
            except BaseException as exc:  # noqa: BLE001
                completion = WorkerCompletion(job=job, error=exc)

            with self._condition:
                self._running = False
                if not self._stopping:
                    self._completions.append(completion)
                self._condition.notify_all()
            if not self._stopping:
                self._completion_guard.trigger()

    def _drain_completions(self) -> None:
        while True:
            with self._condition:
                if not self._completions:
                    return
                completion = self._completions.popleft()
            self._commit(completion)
