import importlib.util
import threading
from pathlib import Path

from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage


PACKAGE = Path(__file__).parents[1]


def load_relay_module():
    path = (
        PACKAGE
        / "scripts"
        / "latest_tf_relay.py"
    )

    spec = importlib.util.spec_from_file_location(
        "latest_tf_relay_under_test",
        path,
    )

    module = importlib.util.module_from_spec(spec)

    assert spec.loader is not None
    spec.loader.exec_module(module)

    return module


def make_transform(parent, child, nanosec):
    transform = TransformStamped()

    transform.header.frame_id = parent
    transform.child_frame_id = child

    transform.header.stamp.sec = 100
    transform.header.stamp.nanosec = nanosec

    return transform


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
        '"history_depth_per_pair"'
        in source
    )

    assert "deque(" in source

    assert (
        "maxlen=self._history_depth_per_pair"
        in source
    )

    assert "self._pending = {}" in source

    assert (
        "self._pending.append("
        not in source
    )


def test_latest_tf_relay_bounds_same_pair_history():
    module = load_relay_module()

    class RelayHarness:
        pass

    relay = RelayHarness()

    relay._lock = threading.Lock()
    relay._history_depth_per_pair = 8
    relay._latest_stamp_ns = {}
    relay._pending = {}

    key = ("map", "odom")

    for nanosec in range(1, 21):
        message = TFMessage()

        message.transforms = [
            make_transform(
                "map",
                "odom",
                nanosec,
            )
        ]

        module.LatestTfRelay._receive(
            relay,
            message,
        )

    assert key in relay._pending
    assert len(relay._pending[key]) == 8

    stamps = [
        module.stamp_nanoseconds(transform)
        for transform in relay._pending[key]
    ]

    expected = [
        100_000_000_000 + nanosec
        for nanosec in range(13, 21)
    ]

    assert stamps == expected

    # Older same-pair data must still be rejected.
    older = TFMessage()
    older.transforms = [
        make_transform(
            "map",
            "odom",
            5,
        )
    ]

    module.LatestTfRelay._receive(
        relay,
        older,
    )

    assert len(relay._pending[key]) == 8

    stamps_after_old = [
        module.stamp_nanoseconds(transform)
        for transform in relay._pending[key]
    ]

    assert stamps_after_old == expected

    class Publisher:
        def __init__(self):
            self.messages = []

        def publish(self, message):
            self.messages.append(message)

    relay._publisher = Publisher()

    module.LatestTfRelay._publish_latest(
        relay
    )

    assert relay._pending == {}
    assert len(relay._publisher.messages) == 1

    published = relay._publisher.messages[0]

    assert len(published.transforms) == 8

    published_stamps = [
        module.stamp_nanoseconds(transform)
        for transform in published.transforms
    ]

    assert published_stamps == expected


def test_latest_tf_relay_filters_navigation_chain():
    module = load_relay_module()

    expected_pairs = {
        ("map", "odom"),
        ("odom", "base_footprint"),
        ("base_footprint", "base_link"),
    }

    assert (
        module.NAVIGATION_FRAME_PAIRS
        == expected_pairs
    )

    class RelayHarness:
        pass

    relay = RelayHarness()

    relay._lock = threading.Lock()
    relay._history_depth_per_pair = 8
    relay._latest_stamp_ns = {}
    relay._pending = {}

    message = TFMessage()

    message.transforms = [
        make_transform(
            "map",
            "odom",
            1,
        ),
        make_transform(
            "odom",
            "base_footprint",
            2,
        ),
        make_transform(
            "base_footprint",
            "base_link",
            3,
        ),
        make_transform(
            "base_link",
            "lf1",
            4,
        ),
        make_transform(
            "lf1",
            "lf2",
            5,
        ),
    ]

    module.LatestTfRelay._receive(
        relay,
        message,
    )

    assert set(relay._pending) == expected_pairs
    assert (
        set(relay._latest_stamp_ns)
        == expected_pairs
    )

    assert (
        ("base_link", "lf1")
        not in relay._pending
    )

    assert (
        ("lf1", "lf2")
        not in relay._pending
    )


def test_latest_tf_relay_filter_is_explicit():
    source = (
        PACKAGE
        / "scripts"
        / "latest_tf_relay.py"
    ).read_text()

    assert (
        '("map", "odom")'
        in source
    )

    assert (
        '("odom", "base_footprint")'
        in source
    )

    assert (
        '("base_footprint", "base_link")'
        in source
    )

    assert (
        "if key not in NAVIGATION_FRAME_PAIRS:"
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
