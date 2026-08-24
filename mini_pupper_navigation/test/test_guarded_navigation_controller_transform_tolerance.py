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
import re


PARAM_FILE = (
    Path(__file__).resolve().parents[1]
    / "param"
    / "mayday_guarded_navigation.yaml"
)


def _follow_path_section(text):
    start = text.index("    FollowPath:\n")
    end = text.index("\nlocal_costmap:\n", start)
    return text[start:end]


def test_guarded_dwb_allows_delayed_map_transform():
    text = PARAM_FILE.read_text(encoding="utf-8")
    follow = _follow_path_section(text)

    assert "plugin: dwb_core::DWBLocalPlanner" in follow

    values = re.findall(
        r"(?m)^\s+transform_tolerance:"
        r"\s*([0-9.]+)\s*$",
        follow,
    )

    assert values == ["0.75"]
