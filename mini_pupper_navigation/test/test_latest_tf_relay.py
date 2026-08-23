from pathlib import Path


PACKAGE = Path(__file__).parents[1]


def test_guarded_runtime_coalesces_dynamic_tf():
    source = (
        PACKAGE
        / "launch"
        / "guarded_navigation.launch.py"
    ).read_text()

    relay = (
        'executable="latest_tf_relay.py"'
    )

    remap = (
        'SetRemap(\n'
        '            src="/tf",\n'
        '            dst="/nav_tf",\n'
        '        )'
    )

    localization = (
        "IncludeLaunchDescription(\n"
        "            PythonLaunchDescriptionSource(\n"
        "                localization_launch"
    )

    assert relay in source
    assert remap in source
    assert localization in source

    assert source.index(relay) < source.index(remap)
    assert source.index(remap) < source.index(localization)

    assert '"input_topic": "/tf"' in source
    assert '"output_topic": "/nav_tf"' in source
    assert '"publish_frequency": 10.0' in source


def test_latest_tf_relay_uses_non_backlogging_input():
    source = (
        PACKAGE
        / "scripts"
        / "latest_tf_relay.py"
    ).read_text()

    assert (
        "ReliabilityPolicy.BEST_EFFORT"
        in source
    )

    assert (
        "ReliabilityPolicy.RELIABLE"
        in source
    )

    assert source.count("depth=1") >= 2

    assert (
        "HistoryPolicy.KEEP_LAST"
        in source
    )

    assert (
        "self._dirty.clear()"
        in source
    )


def test_latest_tf_relay_is_installed():
    source = (
        PACKAGE
        / "CMakeLists.txt"
    ).read_text()

    assert (
        "PROGRAMS scripts/latest_tf_relay.py"
        in source
    )

    assert (
        "DESTINATION lib/${PROJECT_NAME}"
        in source
    )


def test_latest_tf_runtime_dependencies_declared():
    source = (
        PACKAGE
        / "package.xml"
    ).read_text()

    assert (
        "<exec_depend>rclpy</exec_depend>"
        in source
    )

    assert (
        "<exec_depend>tf2_msgs</exec_depend>"
        in source
    )
