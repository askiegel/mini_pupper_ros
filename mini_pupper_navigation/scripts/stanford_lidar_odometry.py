#!/usr/bin/env python3

# SPDX-License-Identifier: Apache-2.0

import math
import time
from collections import deque

import rclpy

from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from tf2_msgs.msg import TFMessage


def wrap_angle(value):
    return math.atan2(
        math.sin(value),
        math.cos(value),
    )


def quaternion_yaw(q):
    return math.atan2(
        2.0 * (
            q.w * q.z
            + q.x * q.y
        ),
        1.0 - 2.0 * (
            q.y * q.y
            + q.z * q.z
        ),
    )


class StanfordLidarOdometry(Node):

    def __init__(self):
        super().__init__(
            "stanford_lidar_odometry"
        )

        self.declare_parameter(
            "input_tf_topic",
            "/stanford_odom/tf",
        )
        self.declare_parameter(
            "output_topic",
            "/odom/stanford",
        )
        self.declare_parameter(
            "source_parent_frame",
            "stanford_lidar_raw",
        )
        self.declare_parameter(
            "source_child_frame",
            "base_link",
        )
        self.declare_parameter(
            "output_parent_frame",
            "odom",
        )
        self.declare_parameter(
            "output_child_frame",
            "base_footprint",
        )
        self.declare_parameter(
            "velocity_window_seconds",
            0.30,
        )
        self.declare_parameter(
            "stationary_window_seconds",
            1.0,
        )
        self.declare_parameter(
            "stationary_translation_m",
            0.015,
        )
        self.declare_parameter(
            "stationary_yaw_rad",
            math.radians(2.0),
        )

        self.input_tf_topic = (
            self.get_parameter(
                "input_tf_topic"
            ).value
        )
        self.output_topic = (
            self.get_parameter(
                "output_topic"
            ).value
        )
        self.source_parent_frame = (
            self.get_parameter(
                "source_parent_frame"
            ).value.strip("/")
        )
        self.source_child_frame = (
            self.get_parameter(
                "source_child_frame"
            ).value.strip("/")
        )
        self.output_parent_frame = (
            self.get_parameter(
                "output_parent_frame"
            ).value
        )
        self.output_child_frame = (
            self.get_parameter(
                "output_child_frame"
            ).value
        )
        self.velocity_window_seconds = float(
            self.get_parameter(
                "velocity_window_seconds"
            ).value
        )
        self.stationary_window_seconds = float(
            self.get_parameter(
                "stationary_window_seconds"
            ).value
        )
        self.stationary_translation_m = float(
            self.get_parameter(
                "stationary_translation_m"
            ).value
        )
        self.stationary_yaw_rad = float(
            self.get_parameter(
                "stationary_yaw_rad"
            ).value
        )

        if self.velocity_window_seconds <= 0.05:
            raise ValueError(
                "velocity_window_seconds must exceed 0.05"
            )

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
        )

        self.publisher = self.create_publisher(
            Odometry,
            self.output_topic,
            qos,
        )

        self.subscription = (
            self.create_subscription(
                TFMessage,
                self.input_tf_topic,
                self.tf_callback,
                qos,
            )
        )

        self.origin = None

        self.velocity_samples = deque()
        self.stationary_samples = deque()

        self.stationary_latched = False

        self.get_logger().info(
            "Stanford LiDAR odometry: "
            f"{self.input_tf_topic} -> "
            f"{self.output_topic}; "
            "physical scan-matching only"
        )

    def tf_callback(self, message):

        for transform in message.transforms:

            parent = (
                transform.header.frame_id
                .strip("/")
            )

            child = (
                transform.child_frame_id
                .strip("/")
            )

            if (
                parent
                != self.source_parent_frame
                or child
                != self.source_child_frame
            ):
                continue

            raw_x = float(
                transform.transform.translation.x
            )
            raw_y = float(
                transform.transform.translation.y
            )
            raw_yaw = quaternion_yaw(
                transform.transform.rotation
            )

            if self.origin is None:
                self.origin = (
                    raw_x,
                    raw_y,
                    raw_yaw,
                )

                self.get_logger().info(
                    "Stanford LiDAR odometry "
                    "origin initialized"
                )

            ox, oy, oyaw = self.origin

            dx = raw_x - ox
            dy = raw_y - oy

            c = math.cos(oyaw)
            s = math.sin(oyaw)

            x = (
                c * dx
                + s * dy
            )

            y = (
                -s * dx
                + c * dy
            )

            yaw = wrap_angle(
                raw_yaw - oyaw
            )

            stamp_seconds = (
                float(transform.header.stamp.sec)
                + float(transform.header.stamp.nanosec)
                * 1e-9
            )

            sample = (
                stamp_seconds,
                x,
                y,
                yaw,
            )

            self.velocity_samples.append(
                sample
            )

            self.stationary_samples.append(
                sample
            )

            cutoff = (
                stamp_seconds
                - self.velocity_window_seconds
            )

            while (
                len(self.velocity_samples) > 2
                and
                self.velocity_samples[1][0]
                < cutoff
            ):
                self.velocity_samples.popleft()

            stationary_cutoff = (
                stamp_seconds
                - self.stationary_window_seconds
            )

            while (
                len(self.stationary_samples) > 2
                and
                self.stationary_samples[1][0]
                < stationary_cutoff
            ):
                self.stationary_samples.popleft()

            vx = 0.0
            vy = 0.0
            wz = 0.0

            if len(self.velocity_samples) >= 2:

                first = self.velocity_samples[0]
                last = self.velocity_samples[-1]

                dt = last[0] - first[0]

                if dt >= 0.05:
                    world_vx = (
                        last[1] - first[1]
                    ) / dt

                    world_vy = (
                        last[2] - first[2]
                    ) / dt

                    heading = last[3]

                    c_heading = math.cos(heading)
                    s_heading = math.sin(heading)

                    vx = (
                        c_heading * world_vx
                        + s_heading * world_vy
                    )

                    vy = (
                        -s_heading * world_vx
                        + c_heading * world_vy
                    )

                    wz = wrap_angle(
                        last[3] - first[3]
                    ) / dt

            #
            # Cartographer can continue making small local-pose
            # corrections for a few seconds after the robot has
            # physically stopped. Do not interpret those bounded
            # zero-net corrections as physical velocity.
            #
            # This gate remains measurement-only: it uses the
            # LiDAR-derived pose history and never cmd_vel.
            #
            if len(self.stationary_samples) >= 2:

                long_first = (
                    self.stationary_samples[0]
                )

                current = (
                    self.stationary_samples[-1]
                )

                long_dt = (
                    current[0]
                    - long_first[0]
                )

                if (
                    long_dt
                    >= self.stationary_window_seconds
                    * 0.90
                ):

                    recent_target = (
                        current[0] - 0.50
                    )

                    recent_first = min(
                        self.stationary_samples,
                        key=lambda sample: abs(
                            sample[0] - recent_target
                        ),
                    )

                    long_dx = (
                        current[1]
                        - long_first[1]
                    )

                    long_dy = (
                        current[2]
                        - long_first[2]
                    )

                    recent_dx = (
                        current[1]
                        - recent_first[1]
                    )

                    recent_dy = (
                        current[2]
                        - recent_first[2]
                    )

                    long_distance = math.hypot(
                        long_dx,
                        long_dy,
                    )

                    recent_distance = math.hypot(
                        recent_dx,
                        recent_dy,
                    )

                    long_dyaw = wrap_angle(
                        current[3]
                        - long_first[3]
                    )

                    recent_dyaw = wrap_angle(
                        current[3]
                        - recent_first[3]
                    )

                    #
                    # Enter stationary using both horizons.
                    #
                    # 2.5 cm over ~1 s plus 1.5 cm over
                    # ~0.5 s corresponds to motion already
                    # near or below Nav2's stopped region.
                    #
                    stationary_evidence = (
                        long_distance <= 0.025
                        and
                        recent_distance <= 0.015
                        and
                        abs(long_dyaw)
                        <= self.stationary_yaw_rad
                        and
                        abs(recent_dyaw)
                        <= self.stationary_yaw_rad * 0.5
                    )

                    if (
                        not self.stationary_latched
                        and stationary_evidence
                    ):
                        self.stationary_latched = True

                    #
                    # Once stationary, a single Cartographer
                    # correction must not reopen velocity.
                    #
                    # Re-enable measured velocity only when
                    # both the 0.5 s and 1.0 s LiDAR motions
                    # show sustained motion in a consistent
                    # direction.
                    #
                    direction_cosine = 0.0

                    if (
                        long_distance > 1e-6
                        and
                        recent_distance > 1e-6
                    ):
                        direction_cosine = (
                            (
                                long_dx * recent_dx
                                + long_dy * recent_dy
                            )
                            /
                            (
                                long_distance
                                * recent_distance
                            )
                        )

                    translation_motion = (
                        long_distance >= 0.025
                        and
                        recent_distance >= 0.015
                        and
                        direction_cosine >= 0.50
                    )

                    rotation_motion = (
                        abs(long_dyaw)
                        >= math.radians(3.0)
                        and
                        abs(recent_dyaw)
                        >= math.radians(1.5)
                        and
                        long_dyaw * recent_dyaw > 0.0
                    )

                    if (
                        self.stationary_latched
                        and
                        (
                            translation_motion
                            or rotation_motion
                        )
                    ):
                        self.stationary_latched = False

                    if self.stationary_latched:
                        vx = 0.0
                        vy = 0.0
                        wz = 0.0

            output = Odometry()

            output.header.stamp = (
                transform.header.stamp
            )

            output.header.frame_id = (
                self.output_parent_frame
            )

            output.child_frame_id = (
                self.output_child_frame
            )

            output.pose.pose.position.x = x
            output.pose.pose.position.y = y
            output.pose.pose.position.z = 0.0

            output.pose.pose.orientation.z = (
                math.sin(yaw / 2.0)
            )

            output.pose.pose.orientation.w = (
                math.cos(yaw / 2.0)
            )

            output.twist.twist.linear.x = vx
            output.twist.twist.linear.y = vy
            output.twist.twist.angular.z = wz

            self.publisher.publish(output)

            return


def main(args=None):

    rclpy.init(args=args)

    node = StanfordLidarOdometry()

    try:
        rclpy.spin(node)

    except (
        KeyboardInterrupt,
        ExternalShutdownException,
    ):
        pass

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
