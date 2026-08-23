#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import TimerAction
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.actions import SetRemap
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    Run guarded Nav2 while Cartographer owns the live map and localization.

    Required existing runtime:
      Cartographer:
        /map
        map -> odom

      Robot bringup:
        odom -> base_link
        /odom
        /scan

    This launch intentionally starts neither AMCL nor map_server.
    """

    package_share = FindPackageShare(
        "mini_pupper_navigation"
    )

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

    disabled_through_poses_tree = PathJoinSubstitution([
        package_share,
        "behavior_trees",
        "mayday_disabled_navigate_through_poses.xml",
    ])

    navigation_params = LaunchConfiguration(
        "navigation_params_file"
    )
    behavior_tree = LaunchConfiguration("behavior_tree")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")

    return LaunchDescription([
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

        Node(
            package="mini_pupper_navigation",
            executable="latest_tf_relay.py",
            name="mapping_navigation_tf_relay",
            output="screen",
            parameters=[{
                "input_topic": "/tf",
                "output_topic": "/nav_tf",
                "publish_frequency": 10.0,
            }],
        ),

        SetRemap(
            src="/tf",
            dst="/nav_tf",
        ),

        TimerAction(
            period=3.0,
            actions=[
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
                    remappings=[
                        ("cmd_vel", "/cmd_vel"),
                    ],
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
                            "default_nav_through_poses_bt_xml":
                                disabled_through_poses_tree,
                        },
                    ],
                ),
                Node(
                    package="nav2_lifecycle_manager",
                    executable="lifecycle_manager",
                    name=(
                        "lifecycle_manager_mapping_navigation"
                    ),
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
            ],
        ),
    ])
