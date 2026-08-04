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
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare('mini_pupper_navigation')

    localization_launch = PathJoinSubstitution([
        package_share,
        'launch',
        'localization.launch.py',
    ])
    localization_params_default = PathJoinSubstitution([
        package_share,
        'param',
        'mayday_localization.yaml',
    ])
    planning_params_default = PathJoinSubstitution([
        package_share,
        'param',
        'mayday_planning.yaml',
    ])

    map_file = LaunchConfiguration('map')
    localization_params = LaunchConfiguration(
        'localization_params_file'
    )
    planning_params = LaunchConfiguration(
        'planning_params_file'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    initial_pose_x = LaunchConfiguration('initial_pose_x')
    initial_pose_y = LaunchConfiguration('initial_pose_y')
    initial_pose_yaw = LaunchConfiguration('initial_pose_yaw')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            description='Absolute path to a validated map YAML file',
        ),
        DeclareLaunchArgument(
            'localization_params_file',
            default_value=localization_params_default,
        ),
        DeclareLaunchArgument(
            'planning_params_file',
            default_value=planning_params_default,
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
        ),
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
        ),
        DeclareLaunchArgument(
            'initial_pose_x',
            default_value='0.0',
        ),
        DeclareLaunchArgument(
            'initial_pose_y',
            default_value='0.0',
        ),
        DeclareLaunchArgument(
            'initial_pose_yaw',
            default_value='0.0',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(localization_launch),
            launch_arguments={
                'map': map_file,
                'params_file': localization_params,
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'initial_pose_x': initial_pose_x,
                'initial_pose_y': initial_pose_y,
                'initial_pose_yaw': initial_pose_yaw,
            }.items(),
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[
                planning_params,
                {'use_sim_time': use_sim_time},
            ],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_planning',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'node_names': ['planner_server'],
            }],
        ),
    ])
