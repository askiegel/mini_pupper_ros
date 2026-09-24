from pathlib import Path
from types import SimpleNamespace

from mini_pupper_stanford.stanford_readiness_probe import (
    FAIL,
    READY,
    WAIT,
    evaluate,
    failure_diagnostic,
    failure_reason,
    poll_process_pid,
    timeout_diagnostic,
)


PROBE = (
    Path(__file__).resolve().parents[1]
    / "mini_pupper_stanford"
    / "stanford_readiness_probe.py"
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


def test_timeout_diagnostic_represents_readiness_wait_states():
    base = dict(
        elapsed=25.0,
        node_count=0,
        cmd_vel_subscription_observed=False,
        parameter_service_ready=False,
        process_pid_request_issued=False,
        process_pid_future_done=False,
        process_pid="unavailable",
        expected_pid=12345,
    )
    never_discovered = timeout_diagnostic(**base)
    assert "elapsed=25.0" in never_discovered
    assert "node_count=0" in never_discovered
    assert "cmd_vel_subscription=no" in never_discovered
    assert "parameter_service_ready=no" in never_discovered
    assert "process_pid_request_issued=no" in never_discovered
    assert "expected_pid=12345" in never_discovered

    subscriber_missing = timeout_diagnostic(**{**base, "node_count": 1})
    assert "node_count=1" in subscriber_missing
    assert "cmd_vel_subscription=no" in subscriber_missing

    parameter_unavailable = timeout_diagnostic(
        **{
            **base,
            "node_count": 1,
            "cmd_vel_subscription_observed": True,
        }
    )
    assert "cmd_vel_subscription=yes" in parameter_unavailable
    assert "parameter_service_ready=no" in parameter_unavailable

    future_pending = timeout_diagnostic(
        **{
            **base,
            "node_count": 1,
            "cmd_vel_subscription_observed": True,
            "parameter_service_ready": True,
            "process_pid_request_issued": True,
            "process_pid": "pending",
        }
    )
    assert "process_pid_request_issued=yes" in future_pending
    assert "process_pid_future_done=no" in future_pending
    assert "process_pid=pending" in future_pending


def test_fatal_readiness_diagnostics_include_specific_reason_and_pid():
    assert failure_reason(["/stanford_cmd_vel", "/stanford_cmd_vel"], None) == (
        "duplicate /stanford_cmd_vel nodes"
    )
    assert failure_reason(["/stanford_cmd_vel"], "malformed") == (
        "malformed process_pid"
    )
    assert failure_reason(["/stanford_cmd_vel"], 7) == "mismatched process_pid"

    mismatch = failure_diagnostic("mismatched process_pid", 7, 8)
    malformed = failure_diagnostic("malformed process_pid", "malformed", 8)
    duplicate = failure_diagnostic("duplicate /stanford_cmd_vel nodes", "unavailable", 8)
    assert "mismatched process_pid" in mismatch and "expected_pid=8" in mismatch
    assert "malformed process_pid" in malformed
    assert "duplicate /stanford_cmd_vel nodes" in duplicate


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


def test_probe_source_has_no_motion_or_ros_cli_behavior():
    text = PROBE.read_text(encoding="utf-8")

    assert "HardwareInterface" not in text
    assert "from geometry_msgs.msg import Twist" not in text
    assert "create_publisher" not in text
    assert ".publish(" not in text
    assert "/motion" not in text
    assert "ROBOT_BRIDGE" not in text
    assert "subprocess" not in text
    assert "ros2 " not in text
