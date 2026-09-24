#!/usr/bin/env python3

import argparse
import os
import time

import rclpy
from rclpy.parameter_client import AsyncParameterClient
from rcl_interfaces.msg import ParameterType


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


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--expected-pid", type=int, required=True)
    parser.add_argument("--timeout", type=float, default=25.0)
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node("stanford_readiness_probe_" + str(os.getpid()))
    client = AsyncParameterClient(node, "/stanford_cmd_vel")
    deadline = time.monotonic() + args.timeout
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=min(0.2, deadline - time.monotonic()))
            nodes = [
                ("/" + name if namespace == "/" else namespace + "/" + name)
                for name, namespace in node.get_node_names_and_namespaces()
            ]
            subs = node.get_subscriber_names_and_types_by_node("stanford_cmd_vel", "/") if "/stanford_cmd_vel" in nodes else []
            future = client.get_parameters(["process_pid"]) if "/stanford_cmd_vel" in nodes else None
            pid = None
            if future is not None:
                rclpy.spin_until_future_complete(node, future, timeout_sec=min(0.2, max(0.0, deadline-time.monotonic())))
                if future.done() and future.result().values:
                    value = future.result().values[0]
                    if value.type == ParameterType.PARAMETER_INTEGER:
                        pid = value.integer_value
                    else:
                        pid = "malformed"
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
