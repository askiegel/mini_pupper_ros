#!/usr/bin/env python3
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
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare(
        "mini_pupper_navigation"
    )

    localization_launch = PathJoinSubstitution([
        package_share,
        "launch",
        "localization.launch.py",
    ])
    localization_params_default = PathJoinSubstitution([
        package_share,
        "param",
        "mayday_localization.yaml",
    ])
    navigation_params_default = PathJoinSubstitution([
        package_share,
        "param",
        "mayday_guarded_navigation.yaml",
    ])
    behavior_tree_default = PathJoinSubstitution([
        package_share,
        "behavior_trees",
        "mayday_guarded_navigate_to_pose.xml",
    ])

    map_file = LaunchConfiguration("map")
    localization_params = LaunchConfiguration(
        "localization_params_file"
    )
    navigation_params = LaunchConfiguration(
        "navigation_params_file"
    )
    behavior_tree = LaunchConfiguration("behavior_tree")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")

    return LaunchDescription([
        DeclareLaunchArgument(
            "map",
            description=(
                "Absolute path to the validated map YAML"
            ),
        ),
        DeclareLaunchArgument(
            "localization_params_file",
            default_value=localization_params_default,
        ),
        DeclareLaunchArgument(
            "navigation_params_file",
            default_value=navigation_params_default,
        ),
        DeclareLaunchArgument(
            "behavior_tree",
            default_value=behavior_tree_default,
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
        ),
        DeclareLaunchArgument(
            "autostart",
            default_value="true",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                localization_launch
            ),
            launch_arguments={
                "map": map_file,
                "params_file": localization_params,
                "use_sim_time": use_sim_time,
                "autostart": autostart,
                "initial_pose_x": "0.0",
                "initial_pose_y": "0.0",
                "initial_pose_yaw": "0.0",
            }.items(),
        ),
        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            output="screen",
            parameters=[
                navigation_params,
                {"use_sim_time": use_sim_time},
            ],
        ),
        Node(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            output="screen",
            parameters=[
                navigation_params,
                {"use_sim_time": use_sim_time},
            ],
            remappings=[("cmd_vel", "/cmd_vel")],
        ),
        Node(
            package="nav2_bt_navigator",
            executable="bt_navigator",
            name="bt_navigator",
            output="screen",
            parameters=[
                navigation_params,
                {
                    "use_sim_time": use_sim_time,
                    "default_nav_to_pose_bt_xml":
                        behavior_tree,
                },
            ],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_guarded_navigation",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "autostart": autostart,
                "bond_timeout": 4.0,
                "attempt_respawn_reconnection": False,
                "node_names": [
                    "planner_server",
                    "controller_server",
                    "bt_navigator",
                ],
            }],
        ),
    ])
