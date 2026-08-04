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

from pathlib import Path
import py_compile

import yaml


PACKAGE = Path(__file__).resolve().parents[1]
LAUNCH = PACKAGE / 'launch' / 'planning.launch.py'
PARAMS = PACKAGE / 'param' / 'mayday_planning.yaml'


def load_params():
    with PARAMS.open(encoding='utf-8') as stream:
        return yaml.safe_load(stream)


def test_launch_has_valid_python_syntax():
    py_compile.compile(str(LAUNCH), doraise=True)


def test_launch_starts_only_localization_and_planning():
    source = LAUNCH.read_text(encoding='utf-8')

    assert 'localization.launch.py' in source
    assert "executable='planner_server'" in source
    assert "name='lifecycle_manager_planning'" in source

    forbidden = (
        'controller_server',
        'bt_navigator',
        'behavior_server',
        'velocity_smoother',
        'rviz2',
    )

    for name in forbidden:
        assert name not in source


def test_planner_configuration_is_conservative():
    planner = load_params()['planner_server']['ros__parameters']
    plugin = planner['GridBased']

    assert planner['use_sim_time'] is False
    assert planner['expected_planner_frequency'] == 1.0
    assert planner['planner_plugins'] == ['GridBased']
    assert plugin['plugin'] == 'nav2_navfn_planner/NavfnPlanner'
    assert plugin['tolerance'] == 0.25
    assert plugin['allow_unknown'] is True


def test_global_costmap_uses_mayday_frames_and_lidar():
    costmap = load_params()['global_costmap']
    costmap = costmap['global_costmap']['ros__parameters']
    scan = costmap['obstacle_layer']['scan']

    assert costmap['use_sim_time'] is False
    assert costmap['global_frame'] == 'map'
    assert costmap['robot_base_frame'] == 'base_link'
    assert costmap['resolution'] == 0.05
    assert costmap['track_unknown_space'] is True
    assert scan['topic'] == '/scan'
    assert scan['data_type'] == 'LaserScan'


def test_costmap_has_only_required_layers():
    costmap = load_params()['global_costmap']
    costmap = costmap['global_costmap']['ros__parameters']

    assert costmap['plugins'] == [
        'static_layer',
        'obstacle_layer',
        'inflation_layer',
    ]

    text = PARAMS.read_text(encoding='utf-8')

    assert 'local_costmap' not in text
    assert '/base/scan' not in text
    assert 'PointCloud2' not in text
    assert 'realsense' not in text
    assert 'zed' not in text


def test_lifecycle_manages_only_planner_server():
    manager = load_params()['lifecycle_manager_planning']
    manager = manager['ros__parameters']

    assert manager['use_sim_time'] is False
    assert manager['autostart'] is True
    assert manager['node_names'] == ['planner_server']


def test_feature_contains_no_motion_control():
    combined = (
        LAUNCH.read_text(encoding='utf-8')
        + PARAMS.read_text(encoding='utf-8')
    )

    forbidden = (
        'controller_server',
        'FollowPath',
        'cmd_vel',
        'bt_navigator',
        'behavior_server',
        'velocity_smoother',
    )

    for name in forbidden:
        assert name not in combined
