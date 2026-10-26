"""Fakes for the ROS objects the planner's tool methods touch: futures, goal handles, an action
client, a publisher, a clock, a logger. Duck-typed, so the same tests run against real messages on Thor."""
import threading
from types import SimpleNamespace as NS

from builtin_interfaces.msg import Time


class FakeFuture:
    """rclpy.task.Future as the planner uses it: add_done_callback fires at once when already done."""

    def __init__(self):
        self._done = False
        self._result = None
        self._callbacks = []
        self._lock = threading.Lock()

    def add_done_callback(self, cb):
        with self._lock:
            if not self._done:
                self._callbacks.append(cb)
                return
        cb(self)

    def set_result(self, result):
        with self._lock:
            self._done = True
            self._result = result
            callbacks, self._callbacks = self._callbacks, []
        for cb in callbacks:
            cb(self)

    def done(self):
        return self._done

    def result(self):
        return self._result


class FakeGoalHandle:
    """The planner's ClientGoalHandle of a running Patrol goal. The cancel is acknowledged after
    `ack_delay_s`; the execute loop only notices it at its next poll (patrol_server sleeps 0.5 s), so
    the result arrives `exit_delay_s` after the cancel, or never when None. Appends to `log` in the
    order things happen."""

    def __init__(self, log, exit_delay_s=0.2, ack_delay_s=0.0, accepted=True):
        self.log = log
        self.exit_delay_s = exit_delay_s
        self.ack_delay_s = ack_delay_s
        self.accepted = accepted
        self.cancel_future = FakeFuture()
        self.result_future = FakeFuture()
        self.result_requests = 0
        self.timers = []

    def _later(self, delay, fn):
        if delay is None:
            return
        t = threading.Timer(delay, fn)
        t.daemon = True
        self.timers.append(t)
        t.start()

    def _ack(self):
        self.log.append('cancel_ack')
        self.cancel_future.set_result(NS(return_code=0))

    def _exit(self):
        self.log.append('old_result')
        self.result_future.set_result(NS(status=5))

    def cancel_goal_async(self):
        self.log.append('cancel')
        self._later(self.ack_delay_s, self._ack)
        self._later(self.exit_delay_s, self._exit)
        return self.cancel_future

    def get_result_async(self):
        self.result_requests += 1
        return self.result_future


class FakeActionClient:
    def __init__(self, log, accepted=True):
        self.log = log
        self.accepted = accepted
        self.goals = []

    def wait_for_server(self, timeout_sec=None):
        return True

    def send_goal_async(self, goal):
        self.log.append('send_goal')
        self.goals.append(goal)
        fut = FakeFuture()
        fut.set_result(FakeGoalHandle(self.log, accepted=self.accepted))
        return fut


class FakePublisher:
    """Records what was published; logs 'path' for a Path with poses and 'path_empty' for the cancel."""

    def __init__(self, log=None, name='path'):
        self.log = log if log is not None else []
        self.name = name
        self.published = []

    def publish(self, msg):
        self.published.append(msg)
        self.log.append(self.name if getattr(msg, 'poses', None) else f'{self.name}_empty')


class FakeClock:
    def now(self):
        return NS(to_msg=lambda: Time(sec=0, nanosec=0))


class FakeLogger:
    def __init__(self):
        self.messages = []

    def __getattr__(self, level):
        if level in ('debug', 'info', 'warn', 'warning', 'error', 'fatal'):
            return lambda msg, **kw: self.messages.append((level, msg))
        raise AttributeError(level)
