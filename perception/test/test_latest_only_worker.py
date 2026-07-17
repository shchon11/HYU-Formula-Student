import threading
import unittest

from hyu_perception.latest_only_worker import LatestOnlyWorker


class _GuardCondition:
    def __init__(self, callback):
        self.callback = callback
        self.triggered = threading.Event()

    def trigger(self):
        self.triggered.set()

    def drain(self):
        self.triggered.clear()
        self.callback()


class _Node:
    def create_guard_condition(self, callback):
        self.guard = _GuardCondition(callback)
        return self.guard


class LatestOnlyWorkerTest(unittest.TestCase):
    def test_running_job_is_kept_and_pending_slot_retains_only_latest(self):
        node = _Node()
        first_started = threading.Event()
        release_first = threading.Event()
        latest_finished = threading.Event()
        computed = []
        committed = []

        def compute(job):
            computed.append(job)
            if job == 1:
                first_started.set()
                self.assertTrue(release_first.wait(timeout=2.0))
            if job == 3:
                latest_finished.set()
            return job * 10

        worker = LatestOnlyWorker(
            node,
            compute,
            committed.append,
            "latest-only-worker-test",
        )
        try:
            self.assertTrue(worker.submit(1))
            self.assertTrue(first_started.wait(timeout=2.0))
            self.assertTrue(worker.submit(2))
            self.assertTrue(worker.submit(3))
            release_first.set()
            self.assertTrue(latest_finished.wait(timeout=2.0))
            self.assertTrue(node.guard.triggered.wait(timeout=2.0))

            node.guard.drain()

            self.assertEqual(computed, [1, 3])
            self.assertEqual(
                [(item.job, item.result, item.error) for item in committed],
                [(1, 10, None), (3, 30, None)],
            )
            self.assertTrue(worker.is_idle())
        finally:
            release_first.set()
            worker.shutdown()

    def test_compute_exception_is_delivered_to_executor_commit(self):
        node = _Node()
        committed = []

        def compute(_job):
            raise RuntimeError("synthetic failure")

        worker = LatestOnlyWorker(
            node,
            compute,
            committed.append,
            "latest-only-worker-error-test",
        )
        try:
            self.assertTrue(worker.submit("job"))
            self.assertTrue(node.guard.triggered.wait(timeout=2.0))
            node.guard.drain()

            self.assertEqual(len(committed), 1)
            self.assertEqual(committed[0].job, "job")
            self.assertIsInstance(committed[0].error, RuntimeError)
        finally:
            worker.shutdown()


if __name__ == "__main__":
    unittest.main()
