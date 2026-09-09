#!/usr/bin/env python3

import json
import os
import sys
import time
from pathlib import Path

import numpy as np

import rclpy

from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String


STANFORD_ROOT = Path(
    "/home/ubuntu/StanfordQuadruped"
)

if not STANFORD_ROOT.is_dir():
    raise RuntimeError(
        f"StanfordQuadruped not found: "
        f"{STANFORD_ROOT}"
    )

sys.path.insert(
    0,
    str(STANFORD_ROOT),
)

from MangDang.mini_pupper.Config import Configuration
from MangDang.mini_pupper.HardwareInterface import HardwareInterface

from pupper.Kinematics import four_legs_inverse_kinematics

from src.Command import Command
from src.Controller import Controller
from src.State import BehaviorState, State


class DummyDisplay:

    def show_state(
        self,
        state,
    ):
        pass


class StanfordCmdVel(Node):

    def __init__(self):

        super().__init__(
            "stanford_cmd_vel"
        )

        self.declare_parameter(
            "dry_run",
            True,
        )

        self.declare_parameter(
            "command_timeout",
            0.40,
        )

        self.declare_parameter(
            "max_linear_x",
            0.15,
        )

        self.declare_parameter(
            "max_linear_y",
            0.0,
        )

        self.declare_parameter(
            "max_angular_z",
            0.20,
        )


        self.dry_run = bool(
            self.get_parameter(
                "dry_run"
            ).value
        )

        self.command_timeout = float(
            self.get_parameter(
                "command_timeout"
            ).value
        )

        self.max_linear_x = float(
            self.get_parameter(
                "max_linear_x"
            ).value
        )

        self.max_linear_y = float(
            self.get_parameter(
                "max_linear_y"
            ).value
        )

        self.max_angular_z = float(
            self.get_parameter(
                "max_angular_z"
            ).value
        )


        if (
            not self.dry_run
            and os.environ.get(
                "MAYDAY_STANFORD_HARDWARE"
            )
            != "1"
        ):
            raise RuntimeError(
                "Hardware mode requires "
                "MAYDAY_STANFORD_HARDWARE=1"
            )


        self.config = Configuration()

        if (
            self.max_linear_x
            > float(
                self.config.max_x_velocity
            )
        ):
            raise RuntimeError(
                "Configured max_linear_x exceeds "
                "Stanford max_x_velocity."
            )


        self.controller = Controller(
            self.config,
            four_legs_inverse_kinematics,
        )

        self.state = State()

        self.state.behavior_state = (
            BehaviorState.REST
        )

        self.state.quat_orientation = (
            np.array(
                [
                    1.0,
                    0.0,
                    0.0,
                    0.0,
                ],
                dtype=float,
            )
        )


        self.command = Command()

        self.display = DummyDisplay()


        self.hardware = None

        if not self.dry_run:

            self.hardware = (
                HardwareInterface()
            )


        self.requested_vx = 0.0
        self.requested_vy = 0.0
        self.requested_wz = 0.0

        self.last_command = (
            time.monotonic()
        )

        self.have_command = False

        self.last_status_publish = 0.0


        self.subscription = (
            self.create_subscription(
                Twist,
                "cmd_vel",
                self.cmd_callback,
                10,
            )
        )

        self.status_publisher = (
            self.create_publisher(
                String,
                "stanford_locomotion/status",
                10,
            )
        )


        self.timer = self.create_timer(
            float(
                self.config.dt
            ),
            self.control_tick,
        )


        self.get_logger().info(
            "Stanford cmd_vel adapter started: "
            f"dry_run={self.dry_run}, "
            f"dt={self.config.dt}, "
            f"max_x={self.max_linear_x}, "
            f"max_wz={self.max_angular_z}"
        )


    @staticmethod
    def clamp(
        value,
        limit,
    ):

        if limit <= 0.0:
            return 0.0

        return max(
            -limit,
            min(
                limit,
                value,
            ),
        )


    def cmd_callback(
        self,
        msg,
    ):

        self.requested_vx = (
            self.clamp(
                float(
                    msg.linear.x
                ),
                self.max_linear_x,
            )
        )

        self.requested_vy = (
            self.clamp(
                float(
                    msg.linear.y
                ),
                self.max_linear_y,
            )
        )

        self.requested_wz = (
            self.clamp(
                float(
                    msg.angular.z
                ),
                self.max_angular_z,
            )
        )

        self.last_command = (
            time.monotonic()
        )

        self.have_command = True


    def effective_command(
        self,
    ):

        stale = (
            not self.have_command
            or (
                time.monotonic()
                - self.last_command
            )
            > self.command_timeout
        )

        if stale:
            return (
                0.0,
                0.0,
                0.0,
                True,
            )

        return (
            self.requested_vx,
            self.requested_vy,
            self.requested_wz,
            False,
        )


    def control_tick(
        self,
    ):

        (
            vx,
            vy,
            wz,
            stale,
        ) = self.effective_command()


        moving = (
            abs(vx) > 0.001
            or abs(vy) > 0.001
            or abs(wz) > 0.001
        )


        currently_trotting = (
            self.state.behavior_state
            == BehaviorState.TROT
        )


        self.command.horizontal_velocity = (
            np.array(
                [
                    vx,
                    vy,
                ],
                dtype=float,
            )
        )

        self.command.yaw_rate = wz

        self.command.hop_event = False
        self.command.activate_event = False


        # Stanford's trot_event is an edge-triggered
        # REST <-> TROT toggle.
        self.command.trot_event = (
            moving
            != currently_trotting
        )


        old_state = (
            self.state.behavior_state
        )


        self.controller.run(
            self.state,
            self.command,
            self.display,
        )


        self.command.trot_event = False


        q = np.asarray(
            self.state.joint_angles,
            dtype=float,
        )


        if q.shape != (3, 4):
            raise RuntimeError(
                f"Unexpected joint shape: "
                f"{q.shape}"
            )

        if not np.all(
            np.isfinite(q)
        ):
            raise RuntimeError(
                "Stanford generated "
                "non-finite joint angles."
            )


        if self.hardware is not None:

            self.hardware.set_actuator_postions(
                q
            )


        if (
            old_state
            != self.state.behavior_state
        ):

            self.get_logger().info(
                "Stanford behavior: "
                f"{old_state} -> "
                f"{self.state.behavior_state}"
            )


        now = time.monotonic()

        if (
            now
            - self.last_status_publish
            >= 0.20
        ):

            status = String()

            status.data = json.dumps({
                "backend":
                    "stanford",
                "dry_run":
                    self.dry_run,
                "behavior":
                    str(
                        self.state.behavior_state
                    ),
                "vx":
                    vx,
                "vy":
                    vy,
                "wz":
                    wz,
                "command_stale":
                    stale,
            })

            self.status_publisher.publish(
                status
            )

            self.last_status_publish = now


    def safe_rest(
        self,
    ):

        self.requested_vx = 0.0
        self.requested_vy = 0.0
        self.requested_wz = 0.0

        self.have_command = True
        self.last_command = (
            time.monotonic()
        )


        count = max(
            1,
            int(
                0.25
                / float(
                    self.config.dt
                )
            ),
        )


        for _ in range(count):

            self.control_tick()

            time.sleep(
                float(
                    self.config.dt
                )
            )


        if self.hardware is not None:

            try:

                self.hardware.pwm_params.esp32.close()

            except Exception:

                pass



def main(
    args=None,
):

    rclpy.init(
        args=args
    )

    node = None

    try:

        node = StanfordCmdVel()

        rclpy.spin(
            node
        )

    except KeyboardInterrupt:

        pass

    finally:

        if node is not None:

            try:

                node.safe_rest()

            except Exception as exc:

                node.get_logger().error(
                    "Safe REST failed: "
                    f"{exc!r}"
                )

            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
