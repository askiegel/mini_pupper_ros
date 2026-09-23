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

LOCALIZATION = (
    PACKAGE
    / "launch"
    / "localization.launch.py"
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


def test_cpp_relay_publishes_nav_tf_on_odometry_updates():
    source = text(SOURCE)

    assert '"/nav_tf"' in source
    assert "create_wall_timer(" not in source
    assert "100ms" not in source
    assert "rclcpp::TimerBase" not in source
    assert source.count("publish_latest();") == 2
    assert "void update_odom(" in source
    assert "void update_local_odom(" in source


def test_cpp_relay_requires_both_odom_samples_before_publish():
    source = text(SOURCE)

    assert "if (!odom || !local_odom)" in source
    assert source.index("if (!odom || !local_odom)") < source.index(
        "publisher_->publish(message);"
    )


def test_cpp_relay_preserves_source_stamps_without_retimestamping():
    source = text(SOURCE)

    assert "transform.header = message.header;" in source
    assert "get_clock()" not in source
    assert ".now()" not in source


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


def test_localization_launch_uses_cpp_relay():
    guarded = text(GUARDED)
    localization = text(LOCALIZATION)

    assert (
        "executable='odometry_nav_tf_relay'"
        in localization
    )

    assert (
        "name='localization_tf_relay'"
        in localization
    )

    assert (
        'executable="latest_tf_relay.py"'
        not in localization
    )

    assert "src='/tf'" in localization
    assert "dst='/nav_tf'" in localization
    assert 'odometry_nav_tf_relay' not in guarded
    assert 'SetRemap' not in guarded


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
