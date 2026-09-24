from pathlib import Path


OWNER = (
    Path(__file__).resolve().parents[1]
    / "scripts"
    / "mayday_stanford_owner"
)

ADAPTER = (
    Path(__file__).resolve().parents[1]
    / "mini_pupper_stanford"
    / "stanford_cmd_vel.py"
)


def source():
    return OWNER.read_text(encoding="utf-8")


def test_adapter_publishes_its_actual_os_pid_as_process_pid_parameter():
    text = ADAPTER.read_text(encoding="utf-8")

    assert '"process_pid"' in text
    assert "os.getpid()" in text
    assert text.index('"process_pid"') < text.index("HardwareInterface()")


def restore_source():
    text = source()
    start = text.index("restore()")
    end = text.index("handle_stop()", start)

    return text[start:end]


def test_owner_has_no_robot_service_dependency():
    text = source()

    for forbidden in (
        "systemctl is-active --quiet robot.service",
        "systemctl stop robot.service",
        "systemctl start robot.service",
        "journalctl -u robot.service",
        "Waiting for L1 to activate robot.",
        "robot.service",
    ):
        assert forbidden not in text


def test_acquisition_keeps_exact_normal_writer_checks():
    text = source()

    assert "find_exactly_one" in text
    assert '"$CHAMP_PATTERN"' in text
    assert '"$SERVO_PATTERN"' in text
    assert "require_unfrozen" in text
    assert text.count('"CHAMP"') >= 2
    assert text.count('"servo_interface"') >= 2


def test_acquisition_keeps_adapter_duplicate_rejection_and_zero_before_freeze():
    text = source()

    adapter_rejection = text.index(
        'fail "Stanford ROS adapter already running."'
    )
    zero = text.index("bridge_stop", adapter_rejection)
    freeze_champ = text.index('kill -STOP "$CHAMP_PID"')
    freeze_servo = text.index('kill -STOP "$SERVO_PID"')

    assert zero < freeze_champ < freeze_servo


def test_acquisition_verifies_freeze_before_adapter_start_and_ready():
    text = source()

    frozen_champ = text.index("require_frozen", text.index('kill -STOP "$SERVO_PID"'))
    frozen_servo = text.index('"servo_interface"', frozen_champ)
    gate = text.index("exclusive Stanford ownership gate established")
    adapter_start = text.index('"$ADAPTER"', gate)
    health = text.index("require_healthy_adapter", adapter_start)
    node = text.index(
        "adapter_node_matches_pid_and_subscribes_to_cmd_vel",
        health,
    )
    ready = text.index("systemd-notify", node)

    assert frozen_champ < frozen_servo < gate < adapter_start
    assert health < node < ready


def test_readiness_is_exact_adapter_node_not_global_subscription_count():
    text = source()

    assert "ros2 topic info" not in text
    assert "Subscription count:" not in text
    assert "adapter_node_matches_pid_and_subscribes_to_cmd_vel" in text
    assert "ros2 node list" in text
    assert "grep -Fx '/stanford_cmd_vel'" in text
    assert "multiple /stanford_cmd_vel nodes found" in text
    assert "ros2 node info /stanford_cmd_vel" in text
    assert '"/cmd_vel:"' in text
    assert '"geometry_msgs/msg/Twist"' in text


def test_readiness_rejects_zombie_or_changed_adapter_before_active_notify():
    text = source()

    health = text.index("require_healthy_adapter()")
    assert '"/proc/$ADAPTER_PID/cmdline"' in text[health:]
    assert "State:[[:space:]]+[XZx]" in text[health:]
    assert "$ADAPTER_PATTERN" in text[health:]

    final_health = text.rindex("require_healthy_adapter")
    active = text.index("Stanford ownership ACTIVE")
    notify = text.index("systemd-notify")

    assert final_health < active < notify


def test_adapter_process_pid_is_bound_to_owner_pid_before_ready():
    text = source()

    assert "ros2 param get" in text
    assert "/stanford_cmd_vel process_pid" in text
    assert '"$node_pid" = "$ADAPTER_PID"' in text
    assert "timeout 2 ros2 node list" in text
    assert "timeout 2 ros2 node info" in text
    assert "timeout 2 ros2 param get" in text
    assert "READINESS_DEADLINE" in text


def test_restore_preserves_zero_adapter_stop_and_normal_writer_order():
    text = restore_source()

    first_zero = text.index("bridge_stop")
    adapter_stop = text.index('kill -INT "$ADAPTER_PID"')
    second_zero = text.index("bridge_stop", first_zero + 1)
    champ_identity = text.index('"/proc/$CHAMP_PID/cmdline"')
    champ_resume = text.index('kill -CONT "$CHAMP_PID"')
    settle = text.index("sleep 1.25")
    servo_identity = text.index('"/proc/$SERVO_PID/cmdline"')
    servo_resume = text.index('kill -CONT "$SERVO_PID"')

    assert first_zero < adapter_stop < second_zero
    assert champ_identity < champ_resume < settle
    assert settle < servo_identity < servo_resume


def test_cleanup_and_systemd_notify_guards_remain():
    text = source()

    assert "trap restore EXIT" in text
    assert "trap handle_stop INT TERM HUP" in text
    assert "MAYDAY_STANFORD_HARDWARE" in text
    assert "command_timeout" in text
    assert "max_linear_x" in text
    assert "max_angular_z" in text
    assert text.index("exclusive Stanford ownership gate established") < text.index(
        "systemd-notify"
    )
