#!/usr/bin/env python3

import argparse
import os
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

        def get_parameters(self, names):
            request = GetParameters.Request()
            request.names = names
            return self._client.call_async(request)


WAIT = "WAIT"
FAIL = "FAIL"
READY = "READY"


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


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--expected-pid", type=int, required=True)
    parser.add_argument("--timeout", type=float, default=25.0)
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node("stanford_readiness_probe_" + str(os.getpid()))
    client = AsyncParameterClient(node, "/stanford_cmd_vel")
    deadline = time.monotonic() + args.timeout
    pending_process_pid = None
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=min(0.2, deadline - time.monotonic()))
            nodes = [
                ("/" + name if namespace == "/" else namespace + "/" + name)
                for name, namespace in node.get_node_names_and_namespaces()
            ]
            subs = node.get_subscriber_names_and_types_by_node("stanford_cmd_vel", "/") if "/stanford_cmd_vel" in nodes else []
            pid = None
            if "/stanford_cmd_vel" in nodes:
                pending_process_pid, pid = poll_process_pid(
                    client, pending_process_pid
                )
            state = evaluate(nodes, subs, pid, args.expected_pid)
            if state == READY:
                return 0
            if state == FAIL:
                return 1
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
