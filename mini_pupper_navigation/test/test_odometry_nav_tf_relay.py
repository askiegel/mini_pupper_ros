from pathlib import Path


PACKAGE = Path(__file__).resolve().parents[1]

SOURCE = (
    PACKAGE
    / "src"
    / "odometry_nav_tf_relay.cpp"
)

CMAKE = PACKAGE / "CMakeLists.txt"
PACKAGE_XML = PACKAGE / "package.xml"

GUARDED = (
    PACKAGE
    / "launch"
    / "guarded_navigation.launch.py"
)

MAPPING = (
    PACKAGE
    / "launch"
    / "mapping_navigation.launch.py"
)


def text(path):
    return path.read_text(
        encoding="utf-8"
    )


def test_cpp_relay_uses_only_odometry_inputs():
    source = text(SOURCE)

    assert '"/odom"' in source
    assert '"/odom/local"' in source

    assert '"/tf"' not in source
    assert '"/tf_static"' not in source


def test_cpp_relay_uses_non_backlogging_input_qos():
    source = text(SOURCE)

    assert (
        "rclcpp::QoS(rclcpp::KeepLast(1))"
        in source
    )

    assert (
        ".best_effort()"
        in source
    )

    assert (
        ".reliable()"
        in source
    )

    assert (
        ".durability_volatile()"
        in source
    )


def test_cpp_relay_publishes_nav_tf_at_ten_hz():
    source = text(SOURCE)

    assert '"/nav_tf"' in source

    assert (
        "create_wall_timer("
        in source
    )

    assert (
        "100ms"
        in source
    )


def test_cpp_relay_outputs_only_two_odom_transforms():
    source = text(SOURCE)

    assert (
        "message.transforms.reserve(2)"
        in source
    )

    assert (
        source.count(
            "message.transforms.push_back("
        )
        == 2
    )

    assert (
        "convert(*odom)"
        in source
    )

    assert (
        "convert(*local_odom)"
        in source
    )


def test_guarded_fixed_map_launch_uses_cpp_relay():
    source = text(GUARDED)

    assert (
        'executable="odometry_nav_tf_relay"'
        in source
    )

    assert (
        'name="guarded_navigation_tf_relay"'
        in source
    )

    assert (
        'executable="latest_tf_relay.py"'
        not in source
    )

    assert 'src="/tf"' in source
    assert 'dst="/nav_tf"' in source


def test_mapping_runtime_keeps_existing_tf_relay():
    source = text(MAPPING)

    assert (
        'executable="latest_tf_relay.py"'
        in source
    )

    assert (
        '"input_topic": "/tf"'
        in source
    )

    assert (
        '"output_topic": "/nav_tf"'
        in source
    )


def test_cpp_relay_is_built_and_installed():
    source = text(CMAKE)

    assert (
        "add_executable("
        in source
    )

    assert (
        "odometry_nav_tf_relay"
        in source
    )

    assert (
        "src/odometry_nav_tf_relay.cpp"
        in source
    )

    assert (
        "ament_target_dependencies("
        in source
    )

    assert (
        "TARGETS odometry_nav_tf_relay"
        in source
    )


def test_cpp_dependencies_are_declared():
    source = text(PACKAGE_XML)

    for dependency in (
        "geometry_msgs",
        "nav_msgs",
        "rclcpp",
        "tf2_msgs",
    ):
        assert (
            f"<build_depend>{dependency}</build_depend>"
            in source
        )

    for dependency in (
        "geometry_msgs",
        "nav_msgs",
        "rclcpp",
        "tf2_msgs",
    ):
        assert (
            f"<exec_depend>{dependency}</exec_depend>"
            in source
        )
