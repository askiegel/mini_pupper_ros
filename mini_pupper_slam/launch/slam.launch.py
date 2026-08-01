#!/usr/bin/env python3
#
# SPDX-License-Identifier: Apache-2.0
#
# Copyright (c) 2022-2023 MangDang
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
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    slam_package = FindPackageShare('mini_pupper_slam')

    cartographer_config_dir = PathJoinSubstitution([
        slam_package,
        'config',
    ])
    cartographer_config_basename = TextSubstitution(
        text='slam.lua'
    )
    rviz_config_file_path = PathJoinSubstitution([
        slam_package,
        'rviz',
        'slam.rviz',
    ])

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')

    use_sim_time_launch_arg = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='false',
        description='Use simulation clock when true',
    )
    use_rviz_launch_arg = DeclareLaunchArgument(
        name='use_rviz',
        default_value='false',
        description='Start RViz when true',
    )

    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
        ],
        arguments=[
            '-configuration_directory',
            cartographer_config_dir,
            '-configuration_basename',
            cartographer_config_basename,
        ],
        remappings=[
            ('/imu/data', 'imu'),
        ],
    )

    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
        ],
        arguments=[
            '-resolution',
            '0.05',
            '-publish_period_sec',
            '1.0',
        ],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=[
            '-d',
            rviz_config_file_path,
        ],
        parameters=[
            {'use_sim_time': use_sim_time},
        ],
        output='screen',
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        use_sim_time_launch_arg,
        use_rviz_launch_arg,
        cartographer_node,
        occupancy_grid_node,
        rviz_node,
    ])
