from pathlib import Path


PACKAGE = Path(__file__).resolve().parents[1]

CONFIG = (
    PACKAGE
    / "config"
    / "stanford_lidar_odometry.lua"
)

ADAPTER = (
    PACKAGE
    / "scripts"
    / "stanford_lidar_odometry.py"
)

LAUNCH = (
    PACKAGE
    / "launch"
    / "stanford_lidar_odometry.launch.py"
)


def text(path):
    return path.read_text(
        encoding="utf-8"
    )


def test_cartographer_is_physical_lidar_only():
    source = text(CONFIG)

    assert "use_odometry = false" in source

    assert (
        "TRAJECTORY_BUILDER_2D."
        "use_imu_data = false"
        in source
    )

    assert (
        "use_online_correlative_scan_matching = true"
        in source
    )

    assert (
        "POSE_GRAPH.optimize_every_n_nodes = 0"
        in source
    )


def test_cartographer_uses_isolated_raw_frame():
    source = text(CONFIG)

    assert (
        'map_frame = "stanford_lidar_raw"'
        in source
    )

    assert (
        'tracking_frame = "base_link"'
        in source
    )

    assert (
        "provide_odom_frame = false"
        in source
    )


def test_adapter_outputs_stanford_odometry():
    source = text(ADAPTER)

    assert '"/stanford_odom/tf"' in source

    assert '"/odom/stanford"' in source

    assert '"odom"' in source

    assert '"base_footprint"' in source

    assert "velocity_window_seconds" in source

    assert "transform.header.stamp.sec" in source
    assert "world_vx" in source
    assert "world_vy" in source
    assert "c_heading" in source
    assert "s_heading" in source


def test_adapter_handles_external_shutdown_cleanly():
    source = text(ADAPTER)

    assert "ExternalShutdownException" in source
    assert "if rclpy.ok():" in source


def test_stationary_gate_uses_only_lidar_pose():
    source = text(ADAPTER)

    assert "stationary_window_seconds" in source
    assert "stationary_yaw_rad" in source

    assert "self.stationary_samples" in source
    assert "self.stationary_latched = False" in source

    assert "long_distance = math.hypot(" in source
    assert "recent_distance = math.hypot(" in source

    assert "stationary_evidence = (" in source

    assert "direction_cosine = 0.0" in source
    assert "translation_motion = (" in source
    assert "rotation_motion = (" in source

    assert "if self.stationary_latched:" in source


def test_stationary_gate_is_latched_and_uses_two_horizons():
    source = text(ADAPTER)

    assert "self.stationary_latched = False" in source

    assert "recent_target = (" in source

    assert "long_distance <= 0.025" in source
    assert "recent_distance <= 0.015" in source

    assert "direction_cosine >= 0.50" in source

    assert "translation_motion = (" in source
    assert "rotation_motion = (" in source

    assert "if self.stationary_latched:" in source


def test_feature_cannot_command_motion():
    source = text(ADAPTER)

    assert '"/cmd_vel"' not in source
    assert "geometry_msgs.msg import Twist" not in source
    assert "create_publisher(Twist" not in source


def test_launch_isolates_cartographer_tf():
    source = text(LAUNCH)

    assert (
        '"/tf",'
        in source
    )

    assert (
        '"/stanford_odom/tf"'
        in source
    )

    assert (
        '"stanford_lidar_cartographer"'
        in source
    )

    assert (
        '"stanford_lidar_odometry"'
        in source
    )

    assert "occupancy_grid" not in source
