#!/usr/bin/env python3

import argparse
import os
import sys
import time

import rclpy


WAIT = "WAIT"
FAIL = "FAIL"
READY = "READY"

NODE_NAME = "/stanford_cmd_vel"
CMD_VEL_SUBSCRIPTION = ("/cmd_vel", ["geometry_msgs/msg/Twist"])
IDENTITY_PREFIX = "/stanford_cmd_vel/process_identity/"
IDENTITY_TYPE = ["std_msgs/msg/Empty"]


def identity_topic(expected_pid):
    return IDENTITY_PREFIX + str(expected_pid)


def identity_markers(publishers):
    return [
        (topic, types)
        for topic, types in publishers
        if topic.startswith(IDENTITY_PREFIX)
    ]


def marker_pid(topic, types):
    suffix = topic.removeprefix(IDENTITY_PREFIX)
    if not suffix.isdecimal() or types != IDENTITY_TYPE:
        return "malformed"
    return int(suffix)


def identity_state(publishers, expected_pid):
    markers = identity_markers(publishers)
    if len(markers) > 1:
        return FAIL, "multiple process identity markers"
    if not markers:
        return WAIT, None
    observed_pid = marker_pid(*markers[0])
    if not isinstance(observed_pid, int):
        return FAIL, "malformed process identity marker"
    if observed_pid != expected_pid:
        return FAIL, "mismatched process identity PID"
    return READY, None


def evaluate(nodes, subscriptions, publishers, expected_pid):
    matches = [node for node in nodes if node == NODE_NAME]
    if len(matches) > 1:
        return FAIL
    if not matches:
        return WAIT
    if CMD_VEL_SUBSCRIPTION not in subscriptions:
        return WAIT
    return identity_state(publishers, expected_pid)[0]


def failure_reason(nodes, publishers, expected_pid):
    if sum(node == NODE_NAME for node in nodes) > 1:
        return "duplicate /stanford_cmd_vel nodes"
    return identity_state(publishers, expected_pid)[1]


def yes_no(value):
    return "yes" if value else "no"


def timeout_diagnostic(
    elapsed,
    node_count,
    cmd_vel_subscription_observed,
    identity_marker_observed,
    observed_identity_marker,
    expected_pid,
):
    return "\n".join(
        (
            "Stanford readiness timeout:",
            f"  elapsed={elapsed:.1f}",
            f"  node_count={node_count}",
            "  cmd_vel_subscription="
            f"{yes_no(cmd_vel_subscription_observed)}",
            "  identity_marker_observed="
            f"{yes_no(identity_marker_observed)}",
            f"  observed_identity_marker={observed_identity_marker}",
            f"  expected_identity_marker={identity_topic(expected_pid)}",
            f"  expected_pid={expected_pid}",
        )
    )


def failure_diagnostic(reason, observed_identity_marker, expected_pid):
    return (
        f"Stanford readiness failed: {reason} "
        "(observed_identity_marker="
        f"{observed_identity_marker}, "
        f"expected_identity_marker={identity_topic(expected_pid)})"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--expected-pid", type=int, required=True)
    parser.add_argument("--timeout", type=float, default=25.0)
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node("stanford_readiness_probe_" + str(os.getpid()))
    started = time.monotonic()
    deadline = started + args.timeout
    node_count = 0
    cmd_vel_subscription_observed = False
    identity_marker_observed = False
    observed_identity_marker = "unavailable"
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(
                node, timeout_sec=min(0.2, deadline - time.monotonic())
            )
            nodes = [
                ("/" + name if namespace == "/" else namespace + "/" + name)
                for name, namespace in node.get_node_names_and_namespaces()
            ]
            node_count = sum(node == NODE_NAME for node in nodes)
            subscriptions = (
                node.get_subscriber_names_and_types_by_node(
                    "stanford_cmd_vel", "/"
                )
                if NODE_NAME in nodes
                else []
            )
            cmd_vel_subscription_observed = (
                cmd_vel_subscription_observed
                or CMD_VEL_SUBSCRIPTION in subscriptions
            )
            publishers = (
                node.get_publisher_names_and_types_by_node(
                    "stanford_cmd_vel", "/"
                )
                if NODE_NAME in nodes
                else []
            )
            markers = identity_markers(publishers)
            if markers:
                identity_marker_observed = True
                observed_identity_marker = markers[-1][0]
            state = evaluate(nodes, subscriptions, publishers, args.expected_pid)
            if state == READY:
                return 0
            if state == FAIL:
                print(
                    failure_diagnostic(
                        failure_reason(nodes, publishers, args.expected_pid),
                        observed_identity_marker,
                        args.expected_pid,
                    ),
                    file=sys.stderr,
                )
                return 1
        print(
            timeout_diagnostic(
                time.monotonic() - started,
                node_count,
                cmd_vel_subscription_observed,
                identity_marker_observed,
                observed_identity_marker,
                args.expected_pid,
            ),
            file=sys.stderr,
        )
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
