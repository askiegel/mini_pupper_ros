from types import SimpleNamespace

from mini_pupper_stanford.stanford_readiness_probe import (
    FAIL,
    READY,
    WAIT,
    evaluate,
    poll_process_pid,
)


def test_readiness_decisions_are_exact_and_pid_bound():
    assert evaluate([], [], None, 1) == WAIT
    assert evaluate(["/stanford_cmd_vel"], [], None, 1) == WAIT
    subs = [("/cmd_vel", ["geometry_msgs/msg/Twist"])]
    assert evaluate(["/stanford_cmd_vel"], subs, None, 1) == WAIT
    assert evaluate(["/stanford_cmd_vel"], subs, "1", 1) == FAIL
    assert evaluate(["/stanford_cmd_vel"], subs, 2, 1) == FAIL
    assert evaluate(["/stanford_cmd_vel"], subs, 1, 1) == READY
    assert evaluate(["/stanford_cmd_vel", "/stanford_cmd_vel"], subs, 1, 1) == FAIL


class FakeFuture:
    def __init__(self, done=False, result=None):
        self._done = done
        self._result = result

    def done(self):
        return self._done

    def result(self):
        return self._result


class FakeClient:
    def __init__(self, future):
        self.future = future
        self.calls = 0

    def get_parameters(self, names):
        assert names == ["process_pid"]
        self.calls += 1
        return self.future


def test_pending_process_pid_request_is_reused_until_it_completes():
    future = FakeFuture()
    client = FakeClient(future)

    pending, pid = poll_process_pid(client, None)
    assert pending is future
    assert pid is None
    assert client.calls == 1

    pending, pid = poll_process_pid(client, pending)
    assert pending is future
    assert pid is None
    assert client.calls == 1


def test_completed_process_pid_request_is_decoded_before_a_new_one_is_started():
    value = SimpleNamespace(type=2, integer_value=42)
    future = FakeFuture(done=True, result=SimpleNamespace(values=[value]))
    client = FakeClient(future)

    pending, pid = poll_process_pid(client, future)
    assert pending is None
    assert pid == 42
    assert client.calls == 0
