#!/usr/bin/env python3

import argparse
import os
import sys
import time

import rclpy
from rcl_interfaces.msg import ParameterType

try:
    from rclpy.parameter_client import AsyncParameterClient
except ImportError:
    from rcl_interfaces.srv import GetParameters

    class AsyncParameterClient:
        """Humble-compatible async client for a node's parameter service."""

        def __init__(self, node, remote_node_name):
            self._client = node.create_client(
                GetParameters, remote_node_name + "/get_parameters"
            )

        def service_is_ready(self):
            return self._client.service_is_ready()

        def get_parameters(self, names):
            request = GetParameters.Request()
            request.names = names
            return self._client.call_async(request)


WAIT = "WAIT"
FAIL = "FAIL"
READY = "READY"


def yes_no(value):
    return "yes" if value else "no"


def timeout_diagnostic(
    elapsed,
    node_count,
    cmd_vel_subscription_observed,
    parameter_service_ready,
    process_pid_request_issued,
    process_pid_future_done,
    process_pid,
    expected_pid,
):
    return "\n".join(
        (
            "Stanford readiness timeout:",
            f"  elapsed={elapsed:.1f}",
            f"  node_count={node_count}",
            "  cmd_vel_subscription="
            f"{yes_no(cmd_vel_subscription_observed)}",
            "  parameter_service_ready="
            f"{yes_no(parameter_service_ready)}",
            "  process_pid_request_issued="
            f"{yes_no(process_pid_request_issued)}",
            "  process_pid_future_done="
            f"{yes_no(process_pid_future_done)}",
            f"  process_pid={process_pid}",
            f"  expected_pid={expected_pid}",
        )
    )


def failure_diagnostic(reason, process_pid, expected_pid):
    return (
        f"Stanford readiness failed: {reason} "
        f"(process_pid={process_pid}, expected_pid={expected_pid})"
    )


def failure_reason(nodes, process_pid):
    matches = [node for node in nodes if node == "/stanford_cmd_vel"]
    if len(matches) > 1:
        return "duplicate /stanford_cmd_vel nodes"
    if not isinstance(process_pid, int):
        return "malformed process_pid"
    return "mismatched process_pid"


def evaluate(nodes, subscriptions, process_pid, expected_pid):
    matches = [node for node in nodes if node == "/stanford_cmd_vel"]
    if len(matches) > 1:
        return FAIL
    if not matches:
        return WAIT
    if ("/cmd_vel", ["geometry_msgs/msg/Twist"]) not in subscriptions:
        return WAIT
    if process_pid is None:
        return WAIT
    if not isinstance(process_pid, int):
        return FAIL
    return READY if process_pid == expected_pid else FAIL


def process_pid_from_future(future):
    try:
        result = future.result()
    except Exception:
        return "malformed"
    if not result.values:
        return None
    value = result.values[0]
    if value.type != ParameterType.PARAMETER_INTEGER:
        return "malformed"
    return value.integer_value


def poll_process_pid(client, future):
    """Return the pending request and its completed process PID, if any."""
    if future is None:
        return client.get_parameters(["process_pid"]), None
    if not future.done():
        return future, None
    return None, process_pid_from_future(future)


def parameter_service_is_ready(client):
    try:
        return client.service_is_ready()
    except Exception:
        return False


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--expected-pid", type=int, required=True)
    parser.add_argument("--timeout", type=float, default=25.0)
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node("stanford_readiness_probe_" + str(os.getpid()))
    client = AsyncParameterClient(node, "/stanford_cmd_vel")
    started = time.monotonic()
    deadline = started + args.timeout
    pending_process_pid = None
    node_count = 0
    cmd_vel_subscription_observed = False
    parameter_service_ready = False
    process_pid_request_issued = False
    process_pid_future_done = False
    last_process_pid = "unavailable"
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=min(0.2, deadline - time.monotonic()))
            nodes = [
                ("/" + name if namespace == "/" else namespace + "/" + name)
                for name, namespace in node.get_node_names_and_namespaces()
            ]
            node_count = sum(node == "/stanford_cmd_vel" for node in nodes)
            subs = (
                node.get_subscriber_names_and_types_by_node(
                    "stanford_cmd_vel", "/"
                )
                if "/stanford_cmd_vel" in nodes
                else []
            )
            cmd_vel_subscription_observed = (
                cmd_vel_subscription_observed
                or ("/cmd_vel", ["geometry_msgs/msg/Twist"]) in subs
            )
            pid = None
            if "/stanford_cmd_vel" in nodes:
                parameter_service_ready = (
                    parameter_service_ready or parameter_service_is_ready(client)
                )
                if pending_process_pid is None:
                    process_pid_request_issued = True
                process_pid_future_done = (
                    process_pid_future_done
                    or (
                        pending_process_pid is not None
                        and pending_process_pid.done()
                    )
                )
                pending_process_pid, pid = poll_process_pid(
                    client, pending_process_pid
                )
                if pending_process_pid is not None:
                    last_process_pid = "pending"
                elif pid is not None:
                    last_process_pid = pid
            state = evaluate(nodes, subs, pid, args.expected_pid)
            if state == READY:
                return 0
            if state == FAIL:
                print(
                    failure_diagnostic(
                        failure_reason(nodes, pid), last_process_pid, args.expected_pid
                    ),
                    file=sys.stderr,
                )
                return 1
        print(
            timeout_diagnostic(
                time.monotonic() - started,
                node_count,
                cmd_vel_subscription_observed,
                parameter_service_ready,
                process_pid_request_issued,
                process_pid_future_done,
                last_process_pid,
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
