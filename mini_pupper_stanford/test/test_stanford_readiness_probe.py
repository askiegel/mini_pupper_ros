from pathlib import Path

from mini_pupper_stanford.stanford_readiness_probe import (
    FAIL,
    READY,
    WAIT,
    CMD_VEL_SUBSCRIPTION,
    IDENTITY_TYPE,
    evaluate,
    failure_diagnostic,
    failure_reason,
    identity_topic,
    timeout_diagnostic,
)


PROBE = (
    Path(__file__).resolve().parents[1]
    / "mini_pupper_stanford"
    / "stanford_readiness_probe.py"
)


def marker(pid, marker_type=IDENTITY_TYPE):
    return (identity_topic(pid), marker_type)


def test_graph_readiness_decisions_are_exact_and_pid_bound():
    nodes = ["/stanford_cmd_vel"]
    subscriptions = [CMD_VEL_SUBSCRIPTION]

    assert evaluate([], [], [], 1) == WAIT
    assert evaluate(nodes, [], [], 1) == WAIT
    assert evaluate(nodes, subscriptions, [], 1) == WAIT
    assert identity_topic(12345) == (
        "/stanford_cmd_vel/process_identity/pid_12345"
    )
    assert evaluate(nodes, subscriptions, [marker(1)], 1) == READY
    assert evaluate(nodes, subscriptions, [marker(2)], 1) == FAIL
    assert evaluate(
        nodes,
        subscriptions,
        [("/stanford_cmd_vel/process_identity/1", IDENTITY_TYPE)],
        1,
    ) == WAIT
    assert evaluate(nodes, subscriptions, [marker(1), marker(2)], 1) == FAIL
    assert evaluate(nodes, subscriptions, [marker("not-a-pid")], 1) == FAIL
    assert evaluate(nodes, subscriptions, [marker(1, ["wrong/type"])], 1) == FAIL
    assert evaluate([*nodes, *nodes], subscriptions, [marker(1)], 1) == FAIL


def test_unrelated_graph_endpoints_do_not_grant_readiness():
    nodes = ["/stanford_cmd_vel"]
    unrelated_subscriptions = [("/other", ["geometry_msgs/msg/Twist"])]
    unrelated_publishers = [
        ("/stanford_cmd_vel/process_identity/1", ["wrong/type"]),
        ("/unrelated/process_identity/1", IDENTITY_TYPE),
    ]

    assert evaluate(nodes, unrelated_subscriptions, [marker(1)], 1) == WAIT
    assert evaluate(nodes, [CMD_VEL_SUBSCRIPTION], unrelated_publishers, 1) == WAIT


def test_timeout_diagnostic_represents_graph_wait_states():
    base = dict(
        elapsed=25.0,
        node_count=0,
        cmd_vel_subscription_observed=False,
        identity_marker_observed=False,
        observed_identity_marker="unavailable",
        expected_pid=12345,
    )
    never_discovered = timeout_diagnostic(**base)
    assert "elapsed=25.0" in never_discovered
    assert "node_count=0" in never_discovered
    assert "cmd_vel_subscription=no" in never_discovered
    assert "identity_marker_observed=no" in never_discovered
    assert "expected_pid=12345" in never_discovered

    subscriber_missing = timeout_diagnostic(**{**base, "node_count": 1})
    assert "node_count=1" in subscriber_missing
    assert "cmd_vel_subscription=no" in subscriber_missing

    marker_missing = timeout_diagnostic(
        **{**base, "node_count": 1, "cmd_vel_subscription_observed": True}
    )
    assert "cmd_vel_subscription=yes" in marker_missing
    assert "identity_marker_observed=no" in marker_missing


def test_fatal_diagnostics_name_identity_failure():
    nodes = ["/stanford_cmd_vel"]
    subscriptions = [CMD_VEL_SUBSCRIPTION]
    assert failure_reason([*nodes, *nodes], [], 8) == (
        "duplicate /stanford_cmd_vel nodes"
    )
    assert failure_reason(nodes, [marker(7), marker(8)], 8) == (
        "multiple process identity markers"
    )
    assert failure_reason(nodes, [marker(7)], 8) == (
        "mismatched process identity PID"
    )
    assert failure_reason(nodes, [marker("bad")], 8) == (
        "malformed process identity marker"
    )

    diagnostic = failure_diagnostic(
        "mismatched process identity PID", identity_topic(7), 8
    )
    assert "mismatched process identity PID" in diagnostic
    assert (
        "expected_identity_marker=/stanford_cmd_vel/process_identity/pid_8"
        in diagnostic
    )


def test_probe_source_has_no_motion_cli_or_parameter_readiness_behavior():
    text = PROBE.read_text(encoding="utf-8")

    for forbidden in (
        "HardwareInterface",
        "from geometry_msgs.msg import Twist",
        "create_publisher",
        ".publish(",
        "/motion",
        "ROBOT_BRIDGE",
        "subprocess",
        "ros2 ",
        "AsyncParameterClient",
        "GetParameters",
        "process_pid",
        "parameter_service",
    ):
        assert forbidden not in text
