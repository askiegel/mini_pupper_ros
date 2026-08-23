#!/usr/bin/env python3

from pathlib import Path


PACKAGE = Path(__file__).resolve().parents[1]
LAUNCH = (
    PACKAGE
    / "launch"
    / "mapping_navigation.launch.py"
)


def source():
    return LAUNCH.read_text(encoding="utf-8")


def test_mapping_navigation_uses_cartographer_runtime():
    text = source()

    assert '"input_topic": "/tf"' in text
    assert '"output_topic": "/nav_tf"' in text
    assert '"publish_frequency": 10.0' in text

    assert 'src="/tf"' in text
    assert 'dst="/nav_tf"' in text

    assert 'package="nav2_planner"' in text
    assert 'executable="planner_server"' in text

    assert 'package="nav2_controller"' in text
    assert 'executable="controller_server"' in text

    assert 'package="nav2_bt_navigator"' in text
    assert 'executable="bt_navigator"' in text


def test_mapping_navigation_starts_no_amcl_or_map_server():
    text = source()

    forbidden = (
        "localization.launch.py",
        "mayday_localization.yaml",
        "nav2_amcl",
        'executable="amcl"',
        "nav2_map_server",
        'executable="map_server"',
        "initial_pose_x",
        "initial_pose_y",
        "initial_pose_yaw",
    )

    for marker in forbidden:
        assert marker not in text


def test_mapping_navigation_reuses_guarded_configuration():
    text = source()

    assert "mayday_guarded_navigation.yaml" in text
    assert "mayday_guarded_navigate_to_pose.xml" in text
    assert "mayday_disabled_navigate_through_poses.xml" in text

    assert '("cmd_vel", "/cmd_vel")' in text


def test_mapping_navigation_lifecycle_owns_only_nav_nodes():
    text = source()

    assert (
        '"lifecycle_manager_mapping_navigation"'
        in text
    )

    required = (
        '"planner_server"',
        '"controller_server"',
        '"bt_navigator"',
    )

    for marker in required:
        assert marker in text


def test_mapping_navigation_waits_for_live_runtime():
    text = source()

    assert "TimerAction(" in text
    assert "period=3.0" in text
