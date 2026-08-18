#
# SPDX-License-Identifier: Apache-2.0
#
# Copyright (c) 2026 Tony Kiegel
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]
LAUNCH = ROOT / "launch" / "guarded_navigation.launch.py"
PARAMS = ROOT / "param" / "mayday_guarded_navigation.yaml"
TREE = (
    ROOT
    / "behavior_trees"
    / "mayday_guarded_navigate_to_pose.xml"
)


def text(path):
    return path.read_text(encoding="utf-8")


def test_guarded_files_exist():
    assert LAUNCH.is_file()
    assert PARAMS.is_file()
    assert TREE.is_file()


def test_launch_is_headless_and_isolated():
    source = text(LAUNCH)

    for marker in (
        'executable="planner_server"',
        'executable="controller_server"',
        'executable="bt_navigator"',
        '"lifecycle_manager_guarded_navigation"',
        '"mayday_guarded_navigation.yaml"',
        '"mayday_guarded_navigate_to_pose.xml"',
    ):
        assert marker in source

    for marker in (
        "rviz2",
        "nav2_bringup",
        "velocity_smoother",
        "waypoint_follower",
        "behavior_server",
        "recoveries_server",
    ):
        assert marker not in source


def test_fixed_low_speed_limits():
    payload = yaml.safe_load(text(PARAMS))
    controller = payload["controller_server"][
        "ros__parameters"
    ]
    follow = controller["FollowPath"]

    assert follow["max_vel_x"] == 0.08
    assert follow["max_speed_xy"] == 0.08
    assert follow["max_vel_theta"] == 0.25
    assert follow["acc_lim_x"] == 0.20
    assert follow["decel_lim_x"] == -0.40
    assert controller["failure_tolerance"] == 0.0
    assert (
        controller["progress_checker"][
            "movement_time_allowance"
        ]
        == 12.0
    )


def test_costmaps_use_only_fixed_lidar():
    payload = yaml.safe_load(text(PARAMS))

    local = payload["local_costmap"][
        "local_costmap"
    ]["ros__parameters"]
    global_map = payload["global_costmap"][
        "global_costmap"
    ]["ros__parameters"]

    assert local["plugins"] == [
        "obstacle_layer",
        "inflation_layer",
    ]
    assert global_map["plugins"] == [
        "static_layer",
        "obstacle_layer",
        "inflation_layer",
    ]

    for costmap in (local, global_map):
        obstacle = costmap["obstacle_layer"]
        scan = obstacle["scan"]

        assert obstacle["observation_sources"] == "scan"
        assert scan["topic"] == "/scan"
        assert scan["data_type"] == "LaserScan"
        assert scan["marking"] is True
        assert scan["clearing"] is True

        serialized = str(costmap).lower()

        for forbidden in (
            "base_scan",
            "realsense",
            "zed",
            "pointcloud2",
        ):
            assert forbidden not in serialized


def test_local_costmap_dimensions_are_integer_parameters():
    payload = yaml.safe_load(text(PARAMS))
    local = payload["local_costmap"][
        "local_costmap"
    ]["ros__parameters"]

    assert local["width"] == 2
    assert local["height"] == 2
    assert type(local["width"]) is int
    assert type(local["height"]) is int
    assert type(local["resolution"]) is float


def test_humble_through_poses_navigator_is_inert():
    source = text(LAUNCH)
    disabled = (
        ROOT
        / "behavior_trees"
        / "mayday_disabled_navigate_through_poses.xml"
    )

    assert disabled.is_file()
    assert (
        '"default_nav_through_poses_bt_xml"'
        in source
    )
    assert (
        '"mayday_disabled_navigate_through_poses.xml"'
        in source
    )

    tree = text(disabled)

    assert tree.count("<AlwaysFailure") == 1

    for forbidden in (
        "ComputePathToPose",
        "ComputePathThroughPoses",
        "FollowPath",
        "RecoveryNode",
        "Spin",
        "BackUp",
        "ClearEntireCostmap",
    ):
        assert forbidden not in tree


def test_tree_has_no_retry_or_recovery():
    source = text(TREE)

    assert source.count("<ComputePathToPose") == 1
    assert source.count("<FollowPath") == 1

    for marker in (
        "RecoveryNode",
        "RoundRobin",
        "RetryUntilSuccessful",
        "Spin",
        "BackUp",
        "ClearEntireCostmap",
        "Wait",
        "RateController",
        "PipelineSequence",
    ):
        assert marker not in source


def test_bt_action_acknowledgement_timeout_is_bounded():
    payload = yaml.safe_load(text(PARAMS))
    navigator = payload["bt_navigator"][
        "ros__parameters"
    ]

    assert navigator["default_server_timeout"] == 5000

    tree = text(TREE)

    assert "server_timeout" not in tree


def test_parameters_exclude_recovery_capability():
    source = text(PARAMS)

    for marker in (
        "behavior_server",
        "recoveries_server",
        "waypoint_follower",
        "velocity_smoother",
        "nav2_spin",
        "nav2_back_up",
        "/base/scan",
        "PointCloud2",
    ):
        assert marker not in source

def test_localization_lifecycle_precedes_guarded_navigation():
    source = text(LAUNCH)

    localization_include = source.index(
        "IncludeLaunchDescription("
    )
    guarded_delay = source.index(
        "TimerAction("
    )
    planner = source.index(
        'executable="planner_server"',
        guarded_delay,
    )
    controller = source.index(
        'executable="controller_server"',
        guarded_delay,
    )
    navigator = source.index(
        'executable="bt_navigator"',
        guarded_delay,
    )
    guarded_manager = source.index(
        '"lifecycle_manager_guarded_navigation"',
        guarded_delay,
    )

    assert (
        "from launch.actions import TimerAction"
        in source
    )
    assert "period=10.0" in source
    assert localization_include < guarded_delay
    assert guarded_delay < planner
    assert guarded_delay < controller
    assert guarded_delay < navigator
    assert planner < guarded_manager
    assert controller < guarded_manager
    assert navigator < guarded_manager

