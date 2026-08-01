#!/usr/bin/env python3
#
# SPDX-License-Identifier: Apache-2.0
#
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
import re
import unittest
from pathlib import Path


PACKAGE_DIR = Path(__file__).resolve().parents[1]
LAUNCH_FILE = PACKAGE_DIR / 'launch' / 'slam.launch.py'


class TestSlamLaunch(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.source = LAUNCH_FILE.read_text(encoding='utf-8')
        ast.parse(cls.source)

    def test_rviz_is_optional_and_disabled_by_default(self):
        declaration = re.search(
            r"DeclareLaunchArgument\(\s*"
            r"name='use_rviz',\s*"
            r"default_value='false'",
            self.source,
            re.DOTALL,
        )

        self.assertIsNotNone(
            declaration,
            'use_rviz must default to false',
        )
        self.assertIn(
            'condition=IfCondition(use_rviz)',
            self.source,
        )

    def test_occupancy_grid_uses_executable_arguments(self):
        node_match = re.search(
            r"occupancy_grid_node = Node\((.*?)\n    \)",
            self.source,
            re.DOTALL,
        )

        self.assertIsNotNone(
            node_match,
            'occupancy-grid node declaration was not found',
        )

        node_source = node_match.group(1)

        self.assertIn("'-resolution'", node_source)
        self.assertIn("'-publish_period_sec'", node_source)
        self.assertIn('arguments=[', node_source)
        self.assertNotIn("{'-resolution':", node_source)
        self.assertNotIn("{'-publish_period_sec':", node_source)

    def test_cartographer_configuration_is_preserved(self):
        self.assertIn(
            'cartographer_config_basename',
            self.source,
        )
        self.assertIn(
            "text='slam.lua'",
            self.source,
        )
        self.assertIn(
            "('/imu/data', 'imu')",
            self.source,
        )


if __name__ == '__main__':
    unittest.main()
