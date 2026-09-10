#!/usr/bin/env python3

# SPDX-License-Identifier: Apache-2.0

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    package_share = FindPackageShare(
        "mini_pupper_navigation"
    )

    config_dir = PathJoinSubstitution([
        package_share,
        "config",
    ])

    config_file = (
        "stanford_lidar_odometry.lua"
    )

    use_sim_time = LaunchConfiguration(
        "use_sim_time"
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
        ),

        Node(
            package="cartographer_ros",
            executable="cartographer_node",
            name="stanford_lidar_cartographer",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
            }],
            arguments=[
                "-configuration_directory",
                config_dir,
                "-configuration_basename",
                config_file,
            ],
            remappings=[
                ("scan", "/scan"),
                (
                    "/tf",
                    "/stanford_odom/tf",
                ),
            ],
        ),

        Node(
            package="mini_pupper_navigation",
            executable=(
                "stanford_lidar_odometry.py"
            ),
            name="stanford_lidar_odometry",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "input_tf_topic":
                    "/stanford_odom/tf",
                "output_topic":
                    "/odom/stanford",
                "source_parent_frame":
                    "stanford_lidar_raw",
                "source_child_frame":
                    "base_link",
                "output_parent_frame":
                    "odom",
                "output_child_frame":
                    "base_footprint",
                "velocity_window_seconds":
                    0.50,
                "stationary_window_seconds":
                    1.0,
                "stationary_translation_m":
                    0.015,
                "stationary_yaw_rad":
                    0.034906585,
            }],
        ),
    ])
