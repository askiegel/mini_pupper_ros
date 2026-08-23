#!/usr/bin/env python3

import copy
import threading

import rclpy

from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from tf2_msgs.msg import TFMessage


def stamp_nanoseconds(transform):
    stamp = transform.header.stamp

    return (
        int(stamp.sec) * 1_000_000_000
        + int(stamp.nanosec)
    )


class LatestTfRelay(Node):
    """
    Coalesce high-rate dynamic TF into one latest-state
    message at a bounded rate for guarded Nav2.

    Input:
        /tf
        BEST_EFFORT
        KEEP_LAST depth 1

    Output:
        /nav_tf
        RELIABLE
        KEEP_LAST depth 1

    Only frame pairs updated since the preceding relay
    publication are emitted. Historical TF is never
    intentionally replayed.
    """

    def __init__(self):
        super().__init__("latest_tf_relay")

        self.declare_parameter(
            "input_topic",
            "/tf",
        )

        self.declare_parameter(
            "output_topic",
            "/nav_tf",
        )

        self.declare_parameter(
            "publish_frequency",
            10.0,
        )

        input_topic = str(
            self.get_parameter(
                "input_topic"
            ).value
        )

        output_topic = str(
            self.get_parameter(
                "output_topic"
            ).value
        )

        frequency = float(
            self.get_parameter(
                "publish_frequency"
            ).value
        )

        if frequency <= 0.0:
            raise ValueError(
                "publish_frequency must be positive"
            )

        input_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        output_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self._lock = threading.Lock()
        self._latest = {}
        self._dirty = set()

        self._publisher = self.create_publisher(
            TFMessage,
            output_topic,
            output_qos,
        )

        self._subscription = self.create_subscription(
            TFMessage,
            input_topic,
            self._receive,
            input_qos,
        )

        self._timer = self.create_timer(
            1.0 / frequency,
            self._publish_latest,
        )

        self.get_logger().info(
            "Latest TF relay: "
            f"{input_topic} -> {output_topic} "
            f"at {frequency:.1f} Hz"
        )

    def _receive(self, message):
        with self._lock:
            for transform in message.transforms:
                key = (
                    transform.header.frame_id,
                    transform.child_frame_id,
                )

                previous = self._latest.get(key)

                if (
                    previous is not None
                    and stamp_nanoseconds(transform)
                    < stamp_nanoseconds(previous)
                ):
                    continue

                self._latest[key] = copy.deepcopy(
                    transform
                )

                self._dirty.add(key)

    def _publish_latest(self):
        with self._lock:
            if not self._dirty:
                return

            keys = sorted(self._dirty)

            transforms = [
                copy.deepcopy(
                    self._latest[key]
                )
                for key in keys
            ]

            self._dirty.clear()

        message = TFMessage()
        message.transforms = transforms

        self._publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)

    node = LatestTfRelay()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
