# Copyright 2026 Tony Kiegel
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

import ast
import importlib.util
from pathlib import Path

import yaml


PACKAGE_DIR = Path(__file__).resolve().parents[1]
LAUNCH_FILE = PACKAGE_DIR / 'launch' / 'localization.launch.py'
PARAM_FILE = (
    PACKAGE_DIR
    / 'param'
    / 'mayday_localization.yaml'
)


def load_launch_module():
    spec = importlib.util.spec_from_file_location(
        'mayday_localization_launch',
        LAUNCH_FILE,
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def node_definitions():
    tree = ast.parse(
        LAUNCH_FILE.read_text(encoding='utf-8')
    )
    nodes = []

    for call in ast.walk(tree):
        if not isinstance(call, ast.Call):
            continue

        if not isinstance(call.func, ast.Name):
            continue

        if call.func.id != 'Node':
            continue

        values = {}

        for keyword in call.keywords:
            if (
                keyword.arg in {
                    'package',
                    'executable',
                    'name',
                }
                and isinstance(keyword.value, ast.Constant)
            ):
                values[keyword.arg] = keyword.value.value

        nodes.append(values)

    return nodes


def test_launch_description_builds():
    module = load_launch_module()
    description = module.generate_launch_description()

    assert description is not None
    assert len(description.entities) == 10


def test_launch_contains_only_localization_nodes():
    assert node_definitions() == [
        {
            'package': 'nav2_map_server',
            'executable': 'map_server',
            'name': 'map_server',
        },
        {
            'package': 'nav2_amcl',
            'executable': 'amcl',
            'name': 'amcl',
        },
        {
            'package': 'nav2_lifecycle_manager',
            'executable': 'lifecycle_manager',
            'name': 'lifecycle_manager_localization',
        },
    ]


def test_launch_requires_explicit_map_and_is_headless():
    source = LAUNCH_FILE.read_text(encoding='utf-8')

    assert "DeclareLaunchArgument(\n            'map'," in source
    assert 'rviz2' not in source
    assert 'controller_server' not in source
    assert 'planner_server' not in source
    assert 'bt_navigator' not in source


def test_amcl_parameters_match_mayday_runtime():
    parameters = yaml.safe_load(
        PARAM_FILE.read_text(encoding='utf-8')
    )
    amcl = parameters['amcl']['ros__parameters']

    assert amcl['use_sim_time'] is False
    assert amcl['base_frame_id'] == 'base_link'
    assert amcl['odom_frame_id'] == 'odom'
    assert amcl['global_frame_id'] == 'map'
    assert amcl['scan_topic'] == '/scan'
    assert amcl['laser_min_range'] == 0.03
    assert amcl['laser_max_range'] == 12.0
    assert amcl['tf_broadcast'] is True
    assert amcl['set_initial_pose'] is True


def test_lifecycle_scope_excludes_navigation_control():
    parameters = yaml.safe_load(
        PARAM_FILE.read_text(encoding='utf-8')
    )
    manager = parameters[
        'lifecycle_manager_localization'
    ]['ros__parameters']

    assert manager['autostart'] is True
    assert manager['node_names'] == [
        'map_server',
        'amcl',
    ]
    assert set(parameters) == {
        'amcl',
        'map_server',
        'lifecycle_manager_localization',
    }
